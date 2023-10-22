import time

import RPi.GPIO as GPIO
import time
import socket
import os
import pigpio

from pwm_servo_utils import general_init, motors_init, servos_init, LED_init, motors_cleanup, general_cleanup, servos_cleanup
from pwm_servo_utils import forward, backward, brake, spin_left, spin_right
from pwm_servo_utils import LED_magenta, LED_blue, LED_off
from pwm_servo_utils import vertical_servo_control, horizontal_servo_control
from pwm_servo_utils import camera_init

LED_R = 22
LED_G = 27
LED_B = 24

class PWM_LED_Manager:
    def __init__(self) -> None:
        general_init()
        LED_init()
        motors_init()

    def cleanup(self):
        GPIO.cleanup()

    def run(self):
        LED_magenta()

if __name__ == '__main__':
    pwm_led_manager = PWM_LED_Manager()
    while True:
        try:
            pwm_led_manager.run()
            time.sleep(0.1)
        except KeyboardInterrupt:
            break
    pwm_led_manager.cleanup()

