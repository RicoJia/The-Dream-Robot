#!/usr/bin/env python3

import pigpio
import time
import rospy

from simple_robotics_python_utils.pubsub.shared_memory_pub_sub import SharedMemorySub
from simple_robotics_python_utils.common.logger import get_logger
from typing import Tuple

IN1 = 20
IN2 = 21
IN3 = 19
IN4 = 26
ENA = 16
ENB = 13


class PigpioMotorControl:
    left_speed = 0.0
    right_speed = 0.0

    def __init__(self):
        self.logger = get_logger(name=self.__class__.__name__)
        self.pi = pigpio.pi()
        self.pi.set_PWM_frequency(ENA, 2000)
        self.pi.set_PWM_frequency(ENB, 2000)
        # Set initial states
        self.pi.write(ENA, 1)
        self.pi.write(IN1, 0)
        self.pi.write(IN2, 0)
        self.pi.write(ENB, 1)
        self.pi.write(IN3, 0)
        self.pi.write(IN4, 0)
        # Start with a duty cycle of 0
        self.pi.set_PWM_dutycycle(ENA, 0)
        self.pi.set_PWM_dutycycle(ENB, 0)
        # pigpio will complain when we want to move the motor
        # after an instance has stopped.
        self.set_motors_enabled()
        self.logger.info(f"{self.__class__.__name__} started")

    def set_motors_disabled(self):
        self.logger.info(f"Motors disabled")
        self.motors_enabled = False

    def set_motors_enabled(self):
        self.logger.info(f"Motors enabled")
        self.motors_enabled = True

    def move_motors(self):
        if self.motors_enabled:
            if self.left_speed > 0:
                self.pi.write(IN1, 1)
                self.pi.write(IN2, 0)
            else:
                self.pi.write(IN1, 0)
                self.pi.write(IN2, 1)
            if self.right_speed > 0:
                self.pi.write(IN3, 1)
                self.pi.write(IN4, 0)
            else:
                self.pi.write(IN3, 0)
                self.pi.write(IN4, 1)
            # Map speed in percentage [0,1] to PWM duty cycle (0-255)
            self.pi.set_PWM_dutycycle(ENA, int(abs(self.left_speed) * 255))
            self.pi.set_PWM_dutycycle(ENB, int(abs(self.right_speed) * 255))
        else:
            self.pi.set_PWM_dutycycle(ENA, 0)
            self.pi.set_PWM_dutycycle(ENB, 0)

    # When done, it's a good idea to stop the pigpio instance
    def cleanup(self):
        self.pi.set_PWM_dutycycle(ENA, 0)
        self.pi.set_PWM_dutycycle(ENB, 0)
        self.set_motors_disabled()
        self.pi.stop()
        self.logger.info(f"{self.__class__.__name__} cleaned up")

    def change_speed(self, speed: Tuple[float, float]):
        if abs(speed[0]) > 1 or abs(speed[1]) > 1:
            self.logger.warning(f"Speed must be in [-1,1]: {speed}")
            return
        self.left_speed, self.right_speed = speed


if __name__ == "__main__":
    rospy.init_node("motor_driver")
    pmc = PigpioMotorControl()
    motor_commands_sub = SharedMemorySub(
        topic=rospy.get_param("/SHM_TOPIC/MOTOR_COMMANDS"),
        data_type=float,
        arr_size=2,
        read_frequency=50,
        callback=pmc.change_speed,
        start_connection_callback=pmc.set_motors_enabled,
        no_connection_callback=pmc.set_motors_disabled,
        debug=False,
    )
    start = time.time()
    stop = False

    last_left = 0
    last_right = 0

    while not rospy.is_shutdown():
        pmc.move_motors()
        rospy.sleep(0.02)

    pmc.cleanup()
