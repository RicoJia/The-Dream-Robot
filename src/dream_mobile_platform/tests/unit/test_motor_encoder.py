#!/usr/bin/env python3

"""
how to run:
1. catkin build
2. catkin test
"""

from unittest.mock import patch, MagicMock
from unittest import TestCase

class MockSHMPub:
    def __init__(self, **kwargs):
        pass

class TestMotorEncoder(TestCase):
    """Notes
    If it's a unit test that HAS to be run through ROS, due to ros imports
    then please write it in unit test. Pytest is not native to ROS
    """
    @patch('dream_mobile_platform.__init__', MagicMock(return_value=None))
    @patch('rospy.get_param', MagicMock(return_value={
        "/PARAMS/WHEEL_DIAMETER": 0.041
    }))
    @patch('simple_robotics_python_utils.pubsub.shared_memory_pub_sub.SharedMemoryPub', MagicMock(return_value=MockSHMPub))
    def test_velocity(self):
        from dream_mobile_platform.motor_encoder import EncoderReader
        import numpy as np
        er = EncoderReader()
        test_encoder_poses = [
            [0.0, 0.0],
            [1.0, 1.0],
            [360.0, 360.0],
            [1.0, 1.0],
        ]
        test_answers = [
            [0.0, 0.0],
            [1.0, 1.0],
            [359.0, 359.0],
            [1.0, 1.0],
        ]
        for test_answer, test_poses in zip(test_answers, test_encoder_poses):
            wheel_diffs = er.get_angle_diffs(test_poses)
            #TODO Remember to remove
            print(f'Rico: {wheel_diffs}')
            self.assertEqual(list(wheel_diffs), test_answer)

        