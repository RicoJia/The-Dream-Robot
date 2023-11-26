#!/usr/bin/env python3
from unittest.mock import MagicMock, patch
from unittest import TestCase

class MockSHMPub:
    def __init__(self, **kwargs):
        pass


# @patch("dream_mobile_platform.__init__", MagicMock(return_value=None))
# @patch("rospy.get_param", MagicMock(return_value={"/PARAMS/WHEEL_DIAMETER": 0.041}))
# @patch(
#     "simple_robotics_python_utils.pubsub.shared_memory_pub_sub.SharedMemoryPub",
#     MagicMock(return_value=MockSHMPub),
# )
class TestIncrementalPIDController(TestCase):
    def test_calc_pid(self):
        # from dream_mobile_platform.motor_controller import IncrementalPIDController
        pass