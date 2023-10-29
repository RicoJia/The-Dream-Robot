from pwm_servo_utils import *
from motor_encoder import WheelEncodersReader, SimpleTestWheelEncoder
import time

general_init()
motors_init()

# wer = WheelEncodersReader()
stwe = SimpleTestWheelEncoder()
start = time.time()
while True:
    try:
        forward(5, 5)
        #TODO Remember to remove
        print(f'Angle: {stwe.get_angle()}')
    except KeyboardInterrupt:
        motors_cleanup()
        break

end = time.time()
print(f"total time: {end-start}")
full_rev = int(input("how many full revolutions?"))
partial_rev = float(input("how many partial revolutions?"))
total_rev = full_rev + partial_rev/14.0
print(f"total revs: {total_rev}") 

# print(f"left right encoder value: ${wer.right_count/total_rev, wer.left_count/total_rev}")

used_pins_cleanup() 

