#!/usr/bin/env python3

"""
how to run:
1. catkin build
2. catkin test
"""

from unittest.mock import patch, MagicMock
from unittest import TestCase
import numpy as np


class MockSHMPub:
    def __init__(self, **kwargs):
        pass


@patch("dream_mobile_platform.__init__", MagicMock(return_value=None))
@patch("rospy.get_param", MagicMock(return_value={"/PARAMS/WHEEL_DIAMETER": 0.041}))
@patch(
    "simple_robotics_python_utils.pubsub.shared_memory_pub_sub.SharedMemoryPub",
    MagicMock(return_value=MockSHMPub),
)
class TestMotorEncoder(TestCase):
    """Notes
    If it's a unit test that HAS to be run through ROS, due to ros imports
    then please write it in unit test. Pytest is not native to ROS
    """

    def test_angle_wrap_wheel_diff(self):
        from dream_mobile_platform.motor_encoder import EncoderReader

        test_diffs = [
            -2 * np.pi,
            -np.pi - np.pi / 2,
            -np.pi,
            -np.pi / 2,
            0.0,
            np.pi / 2,
            np.pi,
            np.pi + np.pi / 2,
            2 * np.pi,
        ]
        test_answers = [
            0.0,
            np.pi / 2,
            -np.pi,
            -np.pi / 2,
            0.0,
            np.pi / 2,
            -np.pi,
            -np.pi / 2,
            0.0,
        ]
        for test_answer, test_diff in zip(test_answers, test_diffs):
            self.assertAlmostEqual(
                test_answer, EncoderReader.angle_wrap_wheel_diff(test_diff), delta=1e-4
            )

    def test_angle_diffs(self):
        from dream_mobile_platform.motor_encoder import EncoderReader

        er = EncoderReader()
        test_encoder_poses_single = [
            0.0,
            np.pi / 2,
            np.pi * 3 / 2,
            np.pi * 2,
            np.pi * 3 / 2,
            np.pi / 2,
            0.0,
        ]
        test_answers_single = [
            0.0,
            np.pi / 2,
            # Note, we want to wrap to [-pi, pi), so it's not np.pi
            -np.pi,
            np.pi / 2,
            -np.pi / 2,
            -np.pi,
            -np.pi / 2,
            -np.pi / 2,
        ]
        for test_pose, test_answer in zip(
            test_encoder_poses_single, test_answers_single
        ):
            test_answers, test_poses = np.ones(2) * test_answer, np.ones(2) * test_pose
            test_results = er.get_angle_diffs(test_poses)
            print(f"Test results: {test_results}, test answers: {test_answers}")
            self.assertTrue(
                np.allclose(test_answers, test_results, rtol=1e-5, atol=1e-8)
            )
