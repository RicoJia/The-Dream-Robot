#!/usr/bin/env python3

import pigpio
import time
import rospy
from dream_mobile_platform.msg import EncoderMsg

import posix_ipc
import mmap
import struct

IN1 = 20
IN2 = 21
IN3 = 19
IN4 = 26
ENA = 16
ENB = 13

LEFT_SH_MEMORY="/LEFT_SH_MEMORY"
RIGHT_SH_MEMORY="/RIGHT_SH_MEMORY"
SH_SIZE = 4096
    
class PigpioMotorControl:
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

    def forward(self, leftspeed, rightspeed):
        self.pi.write(IN1, 1)
        self.pi.write(IN2, 0)
        self.pi.write(IN3, 1)
        self.pi.write(IN4, 0)
        # Map speed (assuming 0-100 input) to PWM duty cycle (0-255)
        self.pi.set_PWM_dutycycle(ENA, int(leftspeed * 255 / 100))
        self.pi.set_PWM_dutycycle(ENB, int(rightspeed * 255 / 100))

    # When done, it's a good idea to stop the pigpio instance
    def cleanup(self):
        self.pi.set_PWM_dutycycle(ENA, 0)
        self.pi.set_PWM_dutycycle(ENB, 0)
        self.pi.stop()
        print(f'{self.__class__.__name__} cleaned up')

if __name__ == "__main__":
    rospy.init_node("test_motor")
    pmc = PigpioMotorControl()

    start = time.time()
    stop = False
    LEFT_PWM = 20
    RIGHT_PWM = 20

    last_left = 0
    last_right = 0

    while not rospy.is_shutdown():
        # forward(LEFT_PWM, RIGHT_PWM)
        pmc.forward(LEFT_PWM, RIGHT_PWM)
        rospy.sleep(0.04)

    # motors_cleanup()
    pmc.cleanup()
