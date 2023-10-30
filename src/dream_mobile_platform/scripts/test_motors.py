from pwm_servo_utils import *
# from motor_encoder import WheelEncodersReader
# from motor_encoder import SimpleTestWheelEncoder
from motor_encoder import PigpioDecoder
import time
import rospy
from dream_mobile_platform.msg import EncoderMsg

general_init()
motors_init()

start = time.time()
stop = False
LEFT_PWM = 10
RIGHT_PWM = 10

last_left = 0
last_right = 0
def cb(msg: EncoderMsg):
    global LEFT_PWM
    global RIGHT_PWM, last_left, last_right
    left, right = msg.left, msg.right
    # #TODO Remember to remove
    # print(f'Rico: {left, right}')
    # if left < last_left:
    #     LEFT_PWM = 0
    # last_left = left
    # if right < last_right:
    #     RIGHT_PWM = 0
    last_right = right

rospy.init_node("test_motor")
sub = rospy.Subscriber("/dream/encoder_status", EncoderMsg, cb)

while not rospy.is_shutdown():
    try:
        forward(LEFT_PWM, RIGHT_PWM)
        rospy.sleep(0.1)

    except KeyboardInterrupt:
        motors_cleanup()
        break
