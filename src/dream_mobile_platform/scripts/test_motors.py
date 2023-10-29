from pwm_servo_utils import *
# from motor_encoder import WheelEncodersReader
# from motor_encoder import SimpleTestWheelEncoder
from motor_encoder import PigpioDecoder
import time

general_init()
motors_init()

start = time.time()
started = False
while True:
    try:
        forward(10, 10)

    except KeyboardInterrupt:
        motors_cleanup()
        break
