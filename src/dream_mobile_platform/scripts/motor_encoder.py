import RPi.GPIO as GPIO
import numpy as np
import enum
from pwm_servo_utils import general_init
import time

LEFT_PHASE_A = 7
RIGHT_PHASE_A = 6
LEFT_PHASE_B = 12
RIGHT_PHASE_B = 17
# encoder ticks, according to the datasheet, it's 374
RIGHT_ENCODER_PULSE_PER_REV = 362 #361.8
# encoder ticks, according to the datasheet, it's 374
LEFT_ENCODER_PULSE_PER_REV = 367 #367.39

##########################################################
# Test Code
##########################################################
# Importing modules and classes
import time
import numpy as np
from gpiozero import RotaryEncoder 
# Assigning parameter values
PPR = 374  # Pulses Per Revolution of the encoder

class SimpleTestWheelEncoder:
    def __init__(self) -> None:
        self.encoder = RotaryEncoder(LEFT_PHASE_A, LEFT_PHASE_B, max_step=0)
        #TODO Remember to remove
        print(f'Rico: {self.encoder} initialized')
    def get_angle(self):
        return 360.0 * self.encoder.steps / PPR

# class MotorMode(enum.Enum):
#     LEFT = enum.auto()
#     RIGHT = enum.auto()

# class WheelEncodersReader:
#     """
#     Right hand rule for motor count is followed. I.e., CCW is positive.
#     """
#     def __init__(self):
#         general_init()
#         self._encoder_measurement_init()
#         self.right_count = 0
#         self.last_right_count = 0
#         self.left_count = 0
#         self.last_left_count = 0

#     def _encoder_measurement_init(self):
#         """Initialize interrupt-driven encoder callbacks"""
#         GPIO.setup(LEFT_PHASE_A,GPIO.IN, pull_up_down=GPIO.PUD_UP)
#         GPIO.setup(RIGHT_PHASE_A,GPIO.IN, pull_up_down=GPIO.PUD_UP)
#         GPIO.setup(LEFT_PHASE_B,GPIO.IN, pull_up_down=GPIO.PUD_UP)
#         GPIO.setup(RIGHT_PHASE_B,GPIO.IN, pull_up_down=GPIO.PUD_UP)
#         GPIO.add_event_detect(RIGHT_PHASE_A, GPIO.RISING, self._rising_edge_encoder_counter)
#         # Note we cannot use LEFT_PHASE_A because of other functions on this pin. 
#         # This is why we need two seaparate encoder counters.
#         GPIO.add_event_detect(LEFT_PHASE_B, GPIO.RISING, self._rising_edge_encoder_counter)
        
#     def _get_encoder_phase_values(self, motor_mode: MotorMode):
#         """ Return a list of left encoder pin readings, either 0 or 1."""
#         if motor_mode == MotorMode.LEFT:
#             return [GPIO.input(LEFT_PHASE_A), GPIO.input(LEFT_PHASE_A)]
#         elif motor_mode == MotorMode.RIGHT:
#             return [GPIO.input(RIGHT_PHASE_A), GPIO.input(RIGHT_PHASE_B)]
#         else:
#             print(f"Please pass in valid MotorMode values: {MotorMode._member_names_}")
#             return []

#     def _rising_edge_encoder_counter(self, pin: int):
#         """
#         Pin-agnostic encoder counter that updates the corresponding pulse counter. 
#         that is, no matter which pins' rising edge are detected,
#         We only care if it's phase A or B. Then, we angle-wrap this value.
#         """
#         start = time.time()
#         # Get phase values and which count based on left or right
#         motor_mode = MotorMode.RIGHT if pin in [RIGHT_PHASE_A, RIGHT_PHASE_B] else MotorMode.LEFT
#         phase_a_val, phase_b_val = self._get_encoder_phase_values(motor_mode)
        
#         # Update increment based on phase A or B.TODO: to find CCW .
#         if pin in [RIGHT_PHASE_A, LEFT_PHASE_A]:
#             increment = -1 if phase_a_val == phase_b_val else 1
#         else:
#             increment = 1 if phase_a_val == phase_b_val else -1
#         if motor_mode == MotorMode.LEFT:
#             self.left_count += increment 
#             # angle wrap -> 1 rev
#         else:
#             self.right_count += increment
#         print(f"{time.time() - start}")

# if __name__ == "__main__":
#     wheel_encoders_publisher = WheelEncodersReader()
#     while True:
#         # right_val = wheel_encoders_publisher._rising_edge_encoder_counter(RIGHT_PHASE_A)
#         # left_val = wheel_encoders_publisher._rising_edge_encoder_counter(LEFT_PHASE_B)
#         # #TODO Remember to remove
#         # print(f'Rico: left {left_val}, right: {right_val}')
#         time.sleep(0.5)
