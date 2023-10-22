#!/usr/bin/env python3

import rospy
from dream_mobile_platform.srv import ToggleLED

from pwm_servo_utils import general_init, servos_init, LED_init, used_pins_cleanup, servos_cleanup
from pwm_servo_utils import set_LED_state

class PWM_LED_Manager:
    def __init__(self) -> None:
        general_init()
        LED_init()
        self.led_server = rospy.Service(rospy.get_param("/PARAM/LED_SERVICE", '/toggle_led'), ToggleLED, self.toggle_led)

    def cleanup(self):
        used_pins_cleanup()
        print(f'Pins have been cleaned up')

    def toggle_led(self, req):
        return set_LED_state(req.state)

if __name__ == '__main__':
    rospy.init_node('pwm_led_manager')
    pwm_led_manager = PWM_LED_Manager()
    rospy.on_shutdown(pwm_led_manager.cleanup)
    rospy.spin()


