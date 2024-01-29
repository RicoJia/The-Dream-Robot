#!/usr/bin/env python3
from sensor_msgs.msg import LaserScan
import unittest
import time

class TestDreamGMapping(unittest.TestCase):
    def test_message_filter(self):
        laser_pub = rospy.Publisher("/scan", LaserScan, queue_size=1)
        # Here we should see dream_gmapping outputting "received first message"
        time.sleep(1)
        

if __name__ == '__main__':
    import rostest
    import rospy
    rospy.init_node("test_dream_gmapping")
    rostest.rosrun("dream_gmapping", "test_dream_gmapping", TestDreamGMapping)

