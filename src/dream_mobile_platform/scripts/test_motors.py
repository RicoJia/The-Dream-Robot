from pwm_servo_utils import *
import time

general_init()
motors_init()


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

used_pins_cleanup() 

