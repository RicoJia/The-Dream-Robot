#!/usr/bin/env python3
from simple_robotics_python_utils.pubsub.shared_memory_pub_sub import (
    SharedMemorySub,
    SharedMemoryPub
)
import rospy
# we want to save params: P, I, D.

if __name__ == "__main__":
    rospy.init_node("test_motors")
    
    encoder_status_sub = SharedMemorySub(
        topic=rospy.get_param("/SHM_TOPIC/WHEEL_VELOCITIES"),
        data_type = float,
        arr_size = 2,
        read_frequency=rospy.get_param("/PARAMS/ENCODER_PUB_FREQUENCY"),
        callback = lambda speeds: print(speeds),
        debug = False
    )

    motor_commands_pub = SharedMemoryPub(
        topic=rospy.get_param("/SHM_TOPIC/MOTOR_COMMANDS"),
        data_type = float,
        arr_size = 2,
        debug = False 
    )

    pwm = 0
    rate = rospy.Rate(rospy.get_param("/PARAMS/MOTOR_PUB_FREQUENCY"))
    while not rospy.is_shutdown():
        pwm = (pwm + 5) % 30
        motor_commands_pub.publish([pwm, pwm])
        rospy.sleep(3)
