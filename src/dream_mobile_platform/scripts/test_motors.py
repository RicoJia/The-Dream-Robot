#!/usr/bin/env python3
from simple_robotics_python_utils.pubsub.shared_memory_pub_sub import (
    SharedMemorySub,
    SharedMemoryPub
)
import rospy

if __name__ == "__main__":
    rospy.init_node("test_motors")
    
    encoder_status_sub = SharedMemorySub(
        topic=rospy.get_param("/SHM_TOPIC/ENCODER_STATUS"),
        data_type = float,
        arr_size = 2,
        read_frequency=10,
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
    while not rospy.is_shutdown():
        pwm = (pwm + 5) % 30
        motor_commands_pub.publish([pwm, pwm])
        #TODO Remember to remove
        print(f'pwm: {pwm}')
        rospy.sleep(3)
