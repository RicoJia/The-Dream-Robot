#!/usr/bin/env python3
from simple_robotics_python_utils.pubsub.pub_sub_utils import Rate
from simple_robotics_python_utils.pubsub.shared_memory_pub_sub import (
    SharedMemorySub,
    SharedMemoryPub,
)
import rospy
from typing import Tuple, Deque, Callable
import numpy as np
from numba import njit
import os

from simple_robotics_python_utils.controllers.pid_controllers import (
   IncrementalPIDController,
   PIDParams 
)

from threading import Lock

LEFT = 0
RIGHT = 1
class MotorOutputRecorder:
    def __init__(self, record_func: Callable[[float, Tuple[float, float]], None]) -> None:
        self._record_func = record_func
        self._pub_sub_lock = Lock()
        self.pwm = 0
        self.encoder_status_sub = SharedMemorySub(
            topic=rospy.get_param("/SHM_TOPIC/WHEEL_VELOCITIES"),
            data_type=float,
            arr_size=2,
            # 50hz is too high 
            read_frequency=10,
            callback=self._record_pwm_and_motor_speeds,
            debug=False,
        )
        # This takes in pwm in [MIN_PWM,MAX_PWM]
        self.motor_commands_pub = SharedMemoryPub(
            topic=rospy.get_param("/SHM_TOPIC/MOTOR_COMMANDS"),
            data_type=float,
            arr_size=2,
            debug=False,
        )


    def _record_pwm_and_motor_speeds(self, motor_speeds: Tuple[float, float]):
        with self._pub_sub_lock:
            self._record_func(self.pwm, motor_speeds)
            

    def pub_new_pwm(self, pwm: float):
        """We publish the same pwm to both motors"""
        with self._pub_sub_lock:
            self.pwm = pwm
            self.motor_commands_pub.publish([self.pwm, self.pwm])

class MotorControlBench:
    """Piece of code that can be run as an individual controller, or easily called in a test bench

    In it, we launch the necessary publishers and subscribers, and of course, the controller itself.
    We provide a step() function so the caller can conveniently controls when to issue a PWM command
    """

    def __init__(self, left_pid_params: PIDParams, right_pid_params: PIDParams) -> None:
        self.pid_controller = IncrementalPIDController(
            left_pid_params, right_pid_params
        )
        self.encoder_status_sub = SharedMemorySub(
            topic=rospy.get_param("/SHM_TOPIC/WHEEL_VELOCITIES"),
            data_type=float,
            arr_size=2,
            read_frequency=rospy.get_param("/PARAMS/ENCODER_PUB_FREQUENCY"),
            callback=self.pid_controller.store_speed,
            start_connection_callback=self.pid_controller.start_getting_wheel_vel_updates,
            no_connection_callback=self.pid_controller.stop_getting_wheel_vel_updates,
            debug=False,
        )

        self.commanded_wheel_vel_sub = SharedMemorySub(
            topic=rospy.get_param("/SHM_TOPIC/COMMANDED_WHEEL_VELOCITY"),
            data_type=float,
            arr_size=2,
            read_frequency=rospy.get_param("/PARAMS/MOTOR_PUB_FREQUENCY"),
            callback=self.pid_controller.store_commanded_speed,
            start_connection_callback=self.pid_controller.start_taking_commands,
            no_connection_callback=self.pid_controller.stop_taking_commands,
            debug=False,
        )

        # This takes in pwm in [-1,1]
        self.motor_commands_pub = SharedMemoryPub(
            topic=rospy.get_param("/SHM_TOPIC/MOTOR_COMMANDS"),
            data_type=float,
            arr_size=2,
            debug=False,
        )
        self.rate = Rate(rospy.get_param("/PARAMS/MOTOR_PUB_FREQUENCY"))

    def get_actual_speeds(self) -> Tuple[float, float]:
        """Convenient function for the user to get the current motor speeds

        Returns:
            Tuple[float, float]: current motor speeds, sp the
        """
        return self.pid_controller.get_actual_speeds()

    def step(self):
        """calculates PWM based on the current motor speeds, and control goal, then publish it.

        Note that publishing the PWM is how we let the motor driver know about our control goal
        Also, we are sleeping according to the rate.
        """
        pwm = self.pid_controller.get_pwms()
        self.motor_commands_pub.publish(list(pwm))
        self.rate.sleep()


if __name__ == "__main__":
    # How to test:
    # 2. add print statement for motor speed as TODO

    rospy.init_node("motor_controller")
    # Find the best PID values
    mcb = MotorControlBench(PIDParams(1, 0.3, 0.0), PIDParams(1, 0, 0))
    while not rospy.is_shutdown():
        mcb.step()
