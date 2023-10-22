#!/usr/bin/env python3

import rospy
from dream_mobile_platform.srv import ToggleLED

from pwm_servo_utils import general_init, motors_init, servos_init, LED_init, motors_cleanup, used_pins_cleanup, servos_cleanup
from pwm_servo_utils import forward, backward, brake, spin_left, spin_right
from pwm_servo_utils import LED_magenta, LED_blue, LED_off
from pwm_servo_utils import vertical_servo_control, horizontal_servo_control
from pwm_servo_utils import camera_init

class PWM_LED_Manager:
    def __init__(self) -> None:
        general_init()
        LED_init()
        motors_init()
        self.led_server = rospy.Service(rospy.get_param("/PARAM/LED_SERVICE", '/toggle_led'), ToggleLED, self.toggle_led)

    def cleanup(self):
        used_pins_cleanup()

    def toggle_led(self, req):
        pass

if __name__ == '__main__':
    rospy.init_node('pwm_led_manager')
    pwm_led_manager = PWM_LED_Manager()
    rospy.on_shutdown(pwm_led_manager.cleanup)
    rospy.spin()


