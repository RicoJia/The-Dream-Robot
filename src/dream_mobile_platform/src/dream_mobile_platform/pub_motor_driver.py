#!/usr/bin/env python3
from simple_robotics_python_utils.pubsub.shared_memory_pub_sub import SharedMemoryPub
from simple_robotics_python_utils.pubsub.pub_sub_utils import Rate

if __name__ == "__main__":
    motor_commands_pub = SharedMemoryPub(
        topic="/motor_commands",
        data_type=float,
        arr_size=2,
        debug=False,
    )
    r = Rate(30)
    i = 0.0
    while True: 
        motor_commands_pub.publish([i, i]) 
        i = (i + 0.1)%1.0
        r.sleep() 