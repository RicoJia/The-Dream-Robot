#!/usr/bin/env python3
from simple_robotics_python_utils.pubsub.pub_sub_utils import Rate
from simple_robotics_python_utils.pubsub.shared_memory_pub_sub import (
    SharedMemorySub,
    SharedMemoryPub
)
import rospy
from collections import deque, namedtuple
from typing import Tuple, Deque
import numpy as np
# from numba import njit

PIDParams = namedtuple("PIDParams", ["kp", "ki", "kd"])

LEFT=0
RIGHT=1
NUM_ERRORS = 2

# we want to save params: P, I, D.
# Generates a list of: pwm, speed 
class IncrementalPIDController:
    def __init__(self, left_params: PIDParams, right_params: PIDParams):
        self.kp = np.array((left_params.kp, right_params.kp)) 
        self.ki = np.array((left_params.ki, right_params.ki))
        self.kd = np.array((left_params.kd, right_params.kd))
        self.stop_getting_wheel_vel_updates()
        self.stop_taking_commands()

   ######################################### Wheel Vel Update Functions #########################################
    def store_speed(self, speed: Tuple[float, float]) -> None:
        """This is a callback for subscribers

        Args:
            speed (Tuple[float, float]): motor speed
        """
        self.motor_speeds = np.asarray(speed)
    
    def start_getting_wheel_vel_updates(self):
        # [e[k-1], e[k-2]]
        self.errors: Deque[np.ndarray] = deque([np.zeros(2) for _ in range(NUM_ERRORS)], maxlen=NUM_ERRORS)
        self.motor_speeds = np.array((0.0, 0.0))
        self.getting_speed_updates = True
        
    def stop_getting_wheel_vel_updates(self):
        self.getting_speed_updates = False
        
   ######################################### Commanded Wheel Vel Functions #########################################
    def store_commanded_speed(self, speed: Tuple[float, float]) -> None:
        """Callback for commanded speed

        Args:
            speed (Tuple[float, float]): motor speed
        """
        self.desired_speeds = np.asarray(speed)

    def start_taking_commands(self):
        self.desired_speeds = np.array((0.0, 0.0)) 
        self.last_pwm: np.ndarray = np.zeros(2)
        self.getting_commanded_wheel_vel = True
    
    def stop_taking_commands(self):
        # TODO
        # self.desired_speeds = np.array((0.5, 0.0)) 
        # self.getting_commanded_wheel_vel = True
        # self.last_pwm: np.ndarray = np.zeros(2)
        self.getting_commanded_wheel_vel = False
        
    def get_pwms(self) -> Tuple[float, float]:
        if self.getting_speed_updates and self.getting_commanded_wheel_vel:
            # e[k] = desired_speeds[k] - motor_speeds[k]
            e = np.asarray(self.desired_speeds) - self.motor_speeds
            current_pwm = IncrementalPIDController.calc_pid(e, self.errors, self.kp, self.ki, self.kd, self.last_pwm)
            # TODO
            print(f'Rico: motor speed: {self.motor_speeds}, pwm: {current_pwm}')
            self.errors.appendleft(e)
            self.last_pwm = current_pwm 
            return tuple(current_pwm)
        else:
            return (0.0, 0.0)
        
    @staticmethod
    # @njitu
    def calc_pid(e, past_errors, kp, ki, kd, last_pwm) -> np.ndarray:
        # u[k] = kp * (e[k] - e[k-1]) + ki * e[k] + kd * (e[k] - 2 * e[k-1] + e[k-2])
        u = kp * (e - past_errors[0]) + ki * e + kd * (e - 2 * past_errors[0] + past_errors[1])
        current_pwm = u + last_pwm
        current_pwm = np.clip(current_pwm, 0.0, 1.0)
        #TODO Remember to remove
        print(f'Rico: e: {e}, u: {u}')
        return current_pwm

class MotorControlBench:
    def __init__(self, left_pid_params: PIDParams, right_pid_params: PIDParams) -> None:
        self.pid_controller = IncrementalPIDController(left_pid_params, right_pid_params) 
        self.encoder_status_sub = SharedMemorySub(
            topic=rospy.get_param("/SHM_TOPIC/WHEEL_VELOCITIES"),
            data_type = float,
            arr_size = 2,
            read_frequency=rospy.get_param("/PARAMS/ENCODER_PUB_FREQUENCY"),
            callback = self.pid_controller.store_speed,
            start_connection_callback = self.pid_controller.start_getting_wheel_vel_updates,
            no_connection_callback = self.pid_controller.stop_getting_wheel_vel_updates,
            debug = False
        )

        self.commanded_wheel_vel_sub = SharedMemorySub(
            topic=rospy.get_param("/SHM_TOPIC/COMMANDED_WHEEL_VELOCITY"),
            data_type=float,
            arr_size=2,
            read_frequency=rospy.get_param("/PARAMS/MOTOR_PUB_FREQUENCY"),
            callback=self.pid_controller.store_commanded_speed,
            start_connection_callback=self.pid_controller.start_taking_commands,
            no_connection_callback=self.pid_controller.stop_taking_commands,
            debug=False
        )

        # This takes in pwm in [0,1]
        self.motor_commands_pub = SharedMemoryPub(
            topic=rospy.get_param("/SHM_TOPIC/MOTOR_COMMANDS"),
            data_type = float,
            arr_size = 2,
            debug = False 
        )
        self.rate = Rate(rospy.get_param("/PARAMS/MOTOR_PUB_FREQUENCY"))

    def step(self):
        pwm = self.pid_controller.get_pwms()
        self.motor_commands_pub.publish(pwm)
        self.rate.sleep()
        
# 1. In an ideal world, we can have a publisher and a subscriber automatically recycled, 
# THat requires: 1. when recycled, it will say bye to its peers. 2. force gc
# 2. Intermediate solution: The instances still exists, but they are unregistered
# 3. Or, We can share pub, and sub. Make them singleton. However, Subscribers needs callback. 
# It's not a good practice to change subscriber callback
# 4. Or, we launch a separate process for the test bench. 
#   - Need: two lists: test data, and timestamp. And that requires multiprocessing.manager, and sharedlist. 
#       Not too bad


if __name__ == "__main__":
    # How to test: 1. store primary speed as TODO; GETTING_COMMANDED_VEL = True 
    # 2. add print statement for motor speed as TODO

    rospy.init_node("motor_controller")
    mcb = MotorControlBench(PIDParams(1,0.3,0.0), PIDParams(1,0,0))
    while not rospy.is_shutdown():
        mcb.step()