#!/usr/bin/env python3
# README: Run this script after source devel setup.bash!
from unittest.mock import MagicMock, patch
from unittest import TestCase
import sys

def mock_get_param(param):
    di = {
        "/SHM_TOPIC/MOTOR_COMMANDS": "TEST_MOTOR_COMMANDS",
        "/SHM_TOPIC/WHEEL_VELOCITIES": "TEST_WHEEL_VELOCITIES",
        "/PARAMS/ENCODER_PUB_FREQUENCY": 2
        }
    return di[param]

class TestMotorOutputRecorder(TestCase):
    """Notes
    1. patch() does not work for patching a function with multiple inputs
    2. import MotorOutputRecorder after patching
    """
    def test_pub_new_pwm(self):
        sys.modules["rospy"] = MagicMock()
        sys.modules["rospy"].get_param = mock_get_param
        from dream_mobile_platform.motor_controller import MotorOutputRecorder
        recorder = MotorOutputRecorder(lambda pwm, speeds: None)
        recorder.pub_new_pwm(1.0)



