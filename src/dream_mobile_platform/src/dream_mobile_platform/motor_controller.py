#!/usr/bin/env python3
from simple_robotics_python_utils.pubsub.pub_sub_utils import Rate
from simple_robotics_python_utils.pubsub.shared_memory_pub_sub import (
    SharedMemorySub,
    SharedMemoryPub,
)
import rospy
from collections import deque, namedtuple
from typing import Tuple, Deque
import numpy as np
from numba import njit

PIDParams = namedtuple("PIDParams", ["kp", "ki", "kd"])

LEFT = 0
RIGHT = 1
NUM_ERRORS = 2


class IncrementalPIDController:
    """Controller for both motors using incremental PID controller"""

    def __init__(self, left_params: PIDParams, right_params: PIDParams):
        self.kp = np.array((left_params.kp, right_params.kp))
        self.ki = np.array((left_params.ki, right_params.ki))
        self.kd = np.array((left_params.kd, right_params.kd))
        self.stop_getting_wheel_vel_updates()
        self.stop_taking_commands()

    ######################################### Wheel Vel Update Functions #########################################
    def store_speed(self, speed: Tuple[float, float]) -> None:
        """Callback for hearing actual speeds

        Args:
            speed (Tuple[float, float]): motor speed
        """
        self.motor_speeds = np.asarray(speed)

    def start_getting_wheel_vel_updates(self):
        """Set flag to True. Relevant params have been reset already"""
        # [e[k-1], e[k-2]]
        self.getting_speed_updates = True

    def stop_getting_wheel_vel_updates(self):
        """Function that resets current speeds and errors and flag."""
        self.getting_speed_updates = False
        self.motor_speeds = np.array((0.0, 0.0))
        self.errors: Deque[np.ndarray] = deque(
            [np.zeros(2) for _ in range(NUM_ERRORS)], maxlen=NUM_ERRORS
        )

    ######################################### Commanded Wheel Vel Functions #########################################
    def store_commanded_speed(self, speed: Tuple[float, float]) -> None:
        """Callback for hearing commanded speed

        Args:
            speed (Tuple[float, float]): motor speed
        """
        self.desired_speeds = np.asarray(speed)

    def start_taking_commands(self):
        """Set flag to True. Relevant params have been reset already"""
        self.getting_commanded_wheel_vel = True

    def stop_taking_commands(self):
        """Function that resets current speeds and pwm and flag."""
        self.desired_speeds = np.array((0.0, 0.0))
        self.last_pwm: np.ndarray = np.zeros(2)
        self.getting_commanded_wheel_vel = False

    def get_pwms(self) -> Tuple[float, float]:
        """High level function to calculate current PWM.

        Returns:
            Tuple[float, float]: PWM for both motors
        """
        if self.getting_speed_updates and self.getting_commanded_wheel_vel:
            # e[k] = desired_speeds[k] - motor_speeds[k]
            e = np.asarray(self.desired_speeds) - self.motor_speeds
            current_pwm = IncrementalPIDController.calc_pid(
                e, self.errors, self.kp, self.ki, self.kd, self.last_pwm
            )
            self.errors.appendleft(e)
            self.last_pwm = current_pwm
            return tuple(current_pwm)
        else:
            return (0.0, 0.0)

    @staticmethod
    # @njit
    def calc_pid(
        e: np.ndarray,
        past_errors: Deque[np.ndarray],
        kp: np.ndarray,
        ki: np.ndarray,
        kd: np.ndarray,
        last_pwm: np.ndarray,
    ) -> np.ndarray:
        """Util function to calculate the current pwm value.

        Args:
            e (np.ndarray): set_point - current_value
            past_errors (np.ndarray): e[k-1], e[k-2]
            kp (np.ndarray): 2-array for both motors
            ki (np.ndarray): 2-array for both motors
            kd (np.ndarray): 2-array for both motors
            last_pwm (np.ndarray): last pwm signal

        Returns:
            np.ndarray: current pwm
        """
        # u[k] = kp * (e[k] - e[k-1]) + ki * e[k] + kd * (e[k] - 2 * e[k-1] + e[k-2])
        u = (
            kp * (e - past_errors[0])
            + ki * e
            + kd * (e - 2 * past_errors[0] + past_errors[1])
        )
        current_pwm = u + last_pwm
        current_pwm = np.clip(current_pwm, 0.0, 1.0)
        return current_pwm

    def get_actual_speeds(self) -> Tuple[float, float]:
        """Small function to get the two motor speeds"""
        return tuple(self.motor_speeds)


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

        # This takes in pwm in [0,1]
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
    mcb = MotorControlBench(PIDParams(1, 0.3, 0.0), PIDParams(1, 0, 0))
    while not rospy.is_shutdown():
        mcb.step()
