import RPi.GPIO as GPIO
import numpy as np


Left_Phase_A = 7      #(left motor phase A)
Right_Phase_A = 6     #(right motor phase A)
Left_Phase_B = 12    #(left motor phase B)
Right_Phase_B = 17 #(right phaseB)
RIGHT_ENCODER_RESOLUTION = 362 #361.8  #(encoder ticks, according to the datasheet, it's 374)
LEFT_ENCODER_RESOLUTION = 367 #367.39  #(encoder ticks, according to the datasheet, it's 374)

def get_left_encoder_vals():
    """ Return a list of left encoder pin readings"""
    left_A = GPIO.input(Left_Phase_A)
    left_B  = GPIO.input(Left_Phase_B)
    return [left_A, left_B]

def get_right_encoder_vals():
    """ Return a list of right encoder pin readings"""
    right_A = GPIO.input(Right_Phase_A)
    right_B = GPIO.input(Right_Phase_B)
    return [right_A, right_B]

class WheelEncodersPublisher:
    def __init__(self):
        self._encoder_measurement_init()

    def _encoder_measurement_init(self):
        GPIO.setup(Left_Phase_A,GPIO.IN, pull_up_down=GPIO.PUD_UP)
        GPIO.setup(Right_Phase_A,GPIO.IN, pull_up_down=GPIO.PUD_UP)
        GPIO.setup(Left_Phase_B,GPIO.IN, pull_up_down=GPIO.PUD_UP)
        GPIO.setup(Right_Phase_B,GPIO.IN, pull_up_down=GPIO.PUD_UP)
        """Initialize interrupt-driven encoder callbacks"""
        GPIO.add_event_detect(Right_Phase_A, GPIO.RISING, self.encoder_counter)
        GPIO.add_event_detect(Left_Phase_B, GPIO.RISING, self.encoder_counter)  #Note we cannot use Left_Phase_A because of other functions on this pin. This is why we need two seaparate encoder counters.
        self.right_count = 0
        self.last_right_count = 0
        self.left_count = 0
        self.last_left_count = 0

    def encoder_counter(self, pin):
        if (pin == Right_Phase_A):
            vals = get_right_encoder_vals()
            # self.right_count += (self.encounter_variation_right(A_val=vals[0], B_val=vals[1]))
            # # Angle wrapping: half angle
            # if np.abs(self.right_count) > 0.5 * RIGHT_ENCODER_RESOLUTION:
            #     self.right_count = np.abs(self.right_count) * (np.abs(self.right_count) - RIGHT_ENCODER_RESOLUTION)
        else:
            vals = get_left_encoder_vals()
            # self.left_count += (self.encounter_variation_left(A_val=vals[0], B_val=vals[1]))
            # # Angle wrapping: half angle
            # if np.abs(self.left_count) > 0.5 * LEFT_ENCODER_RESOLUTION:
            #     self.left_count = np.abs(self.left_count) * (np.abs(self.left_count) - LEFT_ENCODER_RESOLUTION)
        return vals

if __name__ == "__main__":
    import time
    wheel_encoders_publisher = WheelEncodersPublisher()
    while True:
        right_val = wheel_encoders_publisher.encoder_counter(Right_Phase_A)
        left_val = wheel_encoders_publisher.encoder_counter(Left_Phase_B)
        time.sleep(0.5)
