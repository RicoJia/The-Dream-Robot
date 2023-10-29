from pwm_servo_utils import *
# from motor_encoder import WheelEncodersReader
# from motor_encoder import SimpleTestWheelEncoder
from motor_encoder import PigpioDecoder
import time

general_init()
motors_init()

# wer = WheelEncodersReader()
# wer = SimpleTestWheelEncoder()
wer = PigpioDecoder()
start = time.time()
started = False
while True:
    try:
        # if time.time() - start > 1 and not started:
        #     start_val = wer.get_angle()
        #     started = True
        #     print("started")
        # if time.time() - start > 23:
        #     motors_cleanup()
        #     # pass
        #     # end_val = wer.get_angle()
        #     # print("in 2s, number of ticks: ", end_val - start_val)
        #     # break
        # else:
        print(wer.get_angle())
        if wer.get_angle() >= 1496 * 120:
            motors_cleanup()
        else:
            forward(10, 10)

    except KeyboardInterrupt:
        motors_cleanup()
        break

print("final: ", wer.get_angle())
# end = time.time()
# print(f"total time: {end-start}")
# full_rev = int(input("how many full revolutions?"))
# partial_rev = float(input("how many partial revolutions?"))
# total_rev = full_rev + partial_rev/14.0
# print(f"total revs: {total_rev}") 

# # print(f"left right encoder value: ${wer.right_count/total_rev, wer.left_count/total_rev}")

# try:
#     used_pins_cleanup() 
# except Exception:
#     pass

