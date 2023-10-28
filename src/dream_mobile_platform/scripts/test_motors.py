from pwm_servo_utils import *
from motor_encoder import WheelEncodersReader
import time

general_init()
motors_init()

wer = WheelEncodersReader()
start = time.time()
while True:
    try:
        forward(5, 5)
    except KeyboardInterrupt:
        motors_cleanup()
        break

end = time.time()
print(f"total time: {end-start}")
full_rev = int(input("how many full revolutions?"))
partial_rev = int(input("how many partial revolutions?"))
total_rev = full_rev + partial_rev/14.0
print(f"total revs: {total_rev}") 

print(f"left right encoder value: ${wer.right_count, wer.left_count}")

used_pins_cleanup() 

