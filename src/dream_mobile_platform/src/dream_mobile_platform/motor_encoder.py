#!/usr/bin/env python3

import numpy as np

from simple_robotics_python_utils.pubsub.pub_sub_utils import Rate
from simple_robotics_python_utils.pubsub.shared_memory_pub_sub import SharedMemoryPub
import rospy
import typing
import time

def get_pigpio():
    """ we have this funky function so it could make testing easier, since
    pigpio is only installed on rpi
    """
    import pigpio
    return pigpio

LEFT_PHASE_A = 7
RIGHT_PHASE_A = 6
LEFT_PHASE_B = 12
RIGHT_PHASE_B = 17
# This comes from the datasheet, Pulse Per
PPR = 374
# Counts Per Revolution
CPR = PPR * 4
# in meter
WHEEL_DIAMETER = rospy.get_param("/PARAMS/WHEEL_DIAMETER")


##########################################################
# Prod Code
##########################################################
class PigpioDecoder:
    def __init__(self, gpioA, gpioB):
        pigpio = get_pigpio()
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
            self.count = PigpioDecoder.count_wrap(self.count)

    def get_angle(self):
        """Applied after count wrap [0, CPR), return angle in [0, 2pi)"""
        return self.count * 2 * np.pi / CPR

    def cancel(self):
        self.cbA.cancel()
        self.cbB.cancel()

    @staticmethod
    def count_wrap(count):
        """count is in [0, CPR)"""
        return count % CPR

class EncoderReader:
    def __init__(self) -> None:
        self._encoder_pub = SharedMemoryPub(
            topic=rospy.get_param("/SHM_TOPIC/WHEEL_VELOCITIES"),
            data_type=float,
            arr_size=2,
            debug=False
        )
        self.last_wheel_pos_np = np.zeros(2)
        self.last_wheel_time = time.perf_counter()
        print(f"{self.__class__.__name__} has been initialized")

    def angle_wrap_wheel_diff(self,  angle: float):
        """Input angle: [-2pi, 2pi), wrapping angles to [-pi, pi)"""
        diff = (angle + np.pi) % (2 * np.pi) - np.pi
        return -np.pi if diff == np.pi else diff 
    
    def get_angle_diffs(self, current_wheel_angles: typing.List[float]) -> np.ndarray:
        """compute angle-wrapped wheel differences 

        Args:
            current_wheel_angles (typing.List[float]): wheel angles in [0, 2pi)

        Returns:
            typing.List[float]: wheel velocities in [-pi, pi)
        """

        # get_angle_diffs
        current_wheel_angles_np = np.asarray(current_wheel_angles, dtype=float)
        wheel_diffs = current_wheel_angles_np - self.last_wheel_pos_np
        wheel_diffs = np.asarray(list(map(lambda x: self.angle_wrap_wheel_diff(x), wheel_diffs)))
        self.last_wheel_pos_np = current_wheel_angles_np
        return wheel_diffs

    def pub_velocities(self, current_wheel_angles: typing.List[float]):
        """Calculate and publish wheel velocities based on the current wheel angles

        Args:
            current_wheel_angles (typing.List[float]): two wheel angles in radians
        """
        angle_diffs: np.ndarray = self.get_angle_diffs(current_wheel_angles)
        curr_time = time.perf_counter()
        angle_velocities = WHEEL_DIAMETER * angle_diffs/(curr_time - self.last_wheel_time)
        self.last_wheel_time = curr_time
        self._encoder_pub.publish(list(angle_velocities))


if __name__ == "__main__":
    rospy.init_node("~encoder_reader")
    left_decoder = PigpioDecoder(LEFT_PHASE_A, LEFT_PHASE_B)
    right_decoder = PigpioDecoder(RIGHT_PHASE_A, RIGHT_PHASE_B)
    e = EncoderReader()
    r = Rate(50)
    while not rospy.is_shutdown():
        e.pub_velocities(
           [left_decoder.get_angle(), right_decoder.get_angle()] 
        )
        r.sleep()
