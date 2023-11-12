#!/usr/bin/env python3

import pigpio
import time
import rospy

from simple_robotics_python_utils.pubsub.shared_memory_pub_sub import SharedMemorySub
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
        print(f'{self.__class__.__name__} started')

    def move_motors(self):
        self.pi.write(IN1, 1)
        self.pi.write(IN2, 0)
        self.pi.write(IN3, 1)
        self.pi.write(IN4, 0)
        # Map speed (assuming 0-100 input) to PWM duty cycle (0-255)
        self.pi.set_PWM_dutycycle(ENA, int(self.left_speed * 255 / 100))
        self.pi.set_PWM_dutycycle(ENB, int(self.left_speed * 255 / 100))

    # When done, it's a good idea to stop the pigpio instance
    def cleanup(self):
        self.pi.set_PWM_dutycycle(ENA, 0)
        self.pi.set_PWM_dutycycle(ENB, 0)
        self.pi.stop()
        print(f'{self.__class__.__name__} cleaned up')
        
    def change_speed(self, speed: Tuple[float]):
        self.left_speed, self.right_speed = speed

        


if __name__ == "__main__":
    rospy.init_node("motor_driver")
    pmc = PigpioMotorControl()
    motor_commands_sub = SharedMemorySub(
        topic=rospy.get_param("/SHM_TOPIC/MOTOR_COMMANDS"),
        data_type=float,
        arr_size=2,
        read_frequency=50,
        callback = pmc.change_speed,
        debug=False
    )
    start = time.time()
    stop = False

    last_left = 0
    last_right = 0

    while not rospy.is_shutdown():
        # forward(LEFT_PWM, RIGHT_PWM)
        pmc.move_motors()
        rospy.sleep(0.04)

    # motors_cleanup()
    pmc.cleanup()
