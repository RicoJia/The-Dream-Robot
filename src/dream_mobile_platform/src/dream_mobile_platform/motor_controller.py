#!/usr/bin/env python3
"""
sudo_ros_preserve_env rosrun dream_mobile_platform motor_controller.py <_ros_topic_commanded:=true>
_ros_topic_commanded is used when you want to command the node with a ros topic
"""
from simple_robotics_python_utils.pubsub.pub_sub_utils import Rate
from simple_robotics_python_utils.common.logger import get_logger
from simple_robotics_python_utils.pubsub.shared_memory_pub_sub import (
    SharedMemorySub,
    SharedMemoryPub,
)
import rospy
from std_msgs.msg import Float32MultiArray
from typing import Tuple, Deque, Callable
import numpy as np
from numba import njit
import os

from simple_robotics_python_utils.controllers.pid_controllers import (
    BasePIDController,
    IncrementalPIDController,
    PIDParams,
)

from threading import Lock

LEFT = 0
RIGHT = 1


class MotorOutputRecorder:
    def __init__(
        self, record_func: Callable[[float, Tuple[float, float]], None]
    ) -> None:
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

    def __init__(
        self,
        pid_controller: BasePIDController,
        ros_topic_commanded: bool = False,
        debug: bool = False,
    ) -> None:
        self.pid_controller = pid_controller
        self.encoder_status_sub = SharedMemorySub(
            topic=rospy.get_param("/SHM_TOPIC/WHEEL_VELOCITIES"),
            data_type=float,
            arr_size=2,
            read_frequency=rospy.get_param("/PARAMS/ENCODER_PUB_FREQUENCY"),
            callback=self.pid_controller.store_speed,
            start_connection_callback=self.pid_controller.start_getting_wheel_vel_updates,
            no_connection_callback=self.pid_controller.stop_getting_wheel_vel_updates,
            debug=debug,
        )

        if ros_topic_commanded:
            # For testing purposes only
            def ros_topic_commanded_cb(msg):
                # TODO Remember to remove
                print(f"msg: {type(msg.data)}")
                self.pid_controller.store_commanded_speed(msg.data)
                # TODO Remember to remove
                print(
                    f"Rico: {self.pid_controller.getting_speed_updates, self.pid_controller.getting_commanded_wheel_vel, self.pid_controller.get_pwms()}"
                )

            self.pid_controller.start_taking_commands()
            self.commanded_wheel_vel_sub = rospy.Subscriber(
                rospy.get_param("/ROS_TOPIC/COMMANDED_WHEEL_VELOCITY"),
                Float32MultiArray,
                ros_topic_commanded_cb,
            )
        else:
            self.commanded_wheel_vel_sub = SharedMemorySub(
                topic=rospy.get_param("/SHM_TOPIC/COMMANDED_WHEEL_VELOCITY"),
                data_type=float,
                arr_size=2,
                read_frequency=rospy.get_param("/PARAMS/MOTOR_PUB_FREQUENCY"),
                callback=self.pid_controller.store_commanded_speed,
                start_connection_callback=self.pid_controller.start_taking_commands,
                no_connection_callback=self.pid_controller.stop_taking_commands,
                debug=debug,
            )

        # This takes in pwm in [-1,1]
        self.motor_commands_pub = SharedMemoryPub(
            topic=rospy.get_param("/SHM_TOPIC/MOTOR_COMMANDS"),
            data_type=float,
            arr_size=2,
            debug=debug,
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
        #TODO Remember to remove
        # print(f'{rospy.get_time()}, pwm: {pwm}, desired_speeds: {self.pid_controller.desired_speeds}, actual speed: {self.pid_controller.get_actual_speeds()}')
        self.rate.sleep()


if __name__ == "__main__":
    node_name = "motor_controller"
    rospy.init_node(node_name)
    logger = get_logger(node_name)
    logger.info(f"{node_name} has been initialized")
    # Read the best PID values
    pid_controller = IncrementalPIDController(
        PIDParams(kp=0.1749101893319113, ki=0.5571171679053859, kd=0.20820277348684704),
        PIDParams(
            kp=0.20000500208283406, ki=0.7119903002267702, kd=0.20023357363417132
        ),
    )
    ros_topic_commanded = rospy.get_param("~ros_topic_commanded", False)
    mcb = MotorControlBench(pid_controller, ros_topic_commanded)
    while not rospy.is_shutdown():
        mcb.step()
