#!/usr/bin/env python3

import numpy as np
from unittest import TestCase
from unittest.mock import patch, MagicMock
from collections import namedtuple

import sys
import os

# Mocking the dream_mobile_platform module and its classes
sys.modules["dream_mobile_platform"] = MagicMock()
sys.modules["dream_mobile_platform.motor_controller"] = MagicMock()
sys.modules["dream_mobile_platform.motor_controller"].MotorControlBench = MagicMock()
sys.modules["dream_mobile_platform.motor_controller"].PIDParams = namedtuple(
    "PIDParams", ["kp", "ki", "kd"]
)
sys.modules["simple_robotics_python_utils"] = MagicMock()
sys.modules["simple_robotics_python_utils.pubsub"] = MagicMock()
sys.modules["simple_robotics_python_utils.pubsub.shared_memory_pub_sub"] = MagicMock()
sys.modules[
    "simple_robotics_python_utils.pubsub.shared_memory_pub_sub"
].SharedMemoryPub = MagicMock()

# class TestGeneralUtilFunctions(TestCase):
#     def test_score_speed_trajectory(self):
#         from tune_motor_controller import TEST_SEQUENCE, score_speed_trajectory
#         TEST_LENGTH = 3
#         SCORE = 0.1
#         test_data_length_stamps = [(i + 1) * TEST_LENGTH for i in range(len(TEST_SEQUENCE))]
#         test_data = []
#         for setpoint, _ in TEST_SEQUENCE:
#             test_data += [(setpoint + SCORE, setpoint + SCORE)] * TEST_LENGTH
#         score = score_speed_trajectory(test_data_length_stamps, test_data)
#         print(f'test_data_length_stamps: {test_data_length_stamps}, test_data: {test_data}, score: {score}')
#         assert np.allclose(score, SCORE, atol=0.0001)


class TestGeneticAlgorithmPIDTuner(TestCase):
    def setUp(self) -> None:
        from tune_motor_controller import GeneticAlgorithmPIDTuner, PERFORMANCE_FILE

        try:
            os.remove(PERFORMANCE_FILE)
        except FileNotFoundError:
            pass
        self.ga_pid_tuner = GeneticAlgorithmPIDTuner()

    def test_record_and_read_performance(self):
        from dream_mobile_platform.motor_controller import PIDParams

        test_set = [
            [0.1, PIDParams(0.1, 0.1, 0.1), [0.1, 0.1, 0.1, 0.1]],
            [0.2, PIDParams(0.2, 0.2, 0.2), [0.2, 0.2, 0.2, 0.2]],
        ]

        for score, pid, test_data in test_set:
            self.ga_pid_tuner.record_score_and_update_population(score, pid, test_data)
        performances = self.ga_pid_tuner.read_performance_file()
        print("performances: ", performances)
        assert performances == test_set
