#!/usr/bin/env python3

import pigpio
import numpy as np
from pwm_servo_utils import general_init

from simple_robotics_python_utils.pubsub.pub_sub_utils import Rate
from simple_robotics_python_utils.pubsub.shared_memory_pub_sub import SharedMemoryPub
import rospy

LEFT_PHASE_A = 7
RIGHT_PHASE_A = 6
LEFT_PHASE_B = 12
RIGHT_PHASE_B = 17
# This comes from the datasheet, Pulse Per
PPR = 374
# Counts Per Revolution
CPR = PPR * 4
# in meter
WHEEL_DIAMETER = 0.041


##########################################################
# Prod Code
##########################################################
class PigpioDecoder:
    def __init__(self, gpioA, gpioB):
        self.pi = pigpio.pi()
        self.gpioA = gpioA
        self.gpioB = gpioB

        self.levA = 0
        self.levB = 0

        self.lastGpio = None

        self.pi.set_mode(self.gpioA, pigpio.INPUT)
        self.pi.set_mode(self.gpioB, pigpio.INPUT)

        self.pi.set_pull_up_down(self.gpioA, pigpio.PUD_UP)
        self.pi.set_pull_up_down(self.gpioB, pigpio.PUD_UP)

        self.cbA = self.pi.callback(self.gpioA, pigpio.EITHER_EDGE, self._pulse)
        self.cbB = self.pi.callback(self.gpioB, pigpio.EITHER_EDGE, self._pulse)
        self.count = 0

    def _pulse(self, gpio, level, tick):
        """
        Decode the rotary encoder pulse on interrupts, then angle wrap it

                     +---------+         +---------+      0
                     |         |         |         |
           A         |         |         |         |
                     |         |         |         |
           +---------+         +---------+         +----- 1

               +---------+         +---------+            0
               |         |         |         |
           B   |         |         |         |
               |         |         |         |
           ----+         +---------+         +---------+  1
        """
        if gpio == self.gpioA:
            self.levA = level
        else:
            self.levB = level

        if gpio != self.lastGpio:  # debounce
            self.lastGpio = gpio

            if gpio == self.gpioA:
                if level == 1:
                    if self.levB == 1:
                        self.count += 1
                    else:
                        self.count += -1
                else:  # level == 0:
                    if self.levB == 0:
                        self.count += 1
                    else:
                        self.count += -1
            else:  # gpio == self.gpioB
                if level == 1:
                    if self.levA == 1:
                        self.count += -1
                    else:
                        self.count += 1
                else:  # level == 0:
                    if self.levA == 0:
                        self.count += -1
                    else:
                        self.count += 1
            self.count = self._angle_wrap(self.count)

    def get_angle(self):
        return self.count

    def _angle_wrap(self, count):
        return count % CPR

    def cancel(self):
        self.cbA.cancel()
        self.cbB.cancel()


class EncoderReader:
    def __init__(self) -> None:
        self._left_decoder = PigpioDecoder(LEFT_PHASE_A, LEFT_PHASE_B)
        self._right_decoder = PigpioDecoder(RIGHT_PHASE_A, RIGHT_PHASE_B)
        self._encoder_pub = SharedMemoryPub(
            topic=rospy.get_param("/SHM_TOPIC/ENCODER_STATUS"),
            data_type=float,
            arr_size=2,
<<<<<<< HEAD
            verbose=True,
=======
            debug=False
>>>>>>> c9e1d888ef1d89690f94e25c38ef65e0c5868608
        )
        print(f"{self.__class__.__name__} has been initialized")

    def pub(self):
        self._encoder_pub.publish(
            [self._left_decoder.get_angle(), self._right_decoder.get_angle()]
        )


if __name__ == "__main__":
    e = EncoderReader()
    rospy.init_node("~encoder_reader")
    r = Rate(50)
    while not rospy.is_shutdown():
        e.pub()
        r.sleep()
