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


class TestGeneralUtilFunctions(TestCase):
    def test_score_speed_trajectory(self):
        from tune_motor_controller import TEST_SEQUENCE, score_speed_trajectory

        TEST_LENGTH = 3
        SCORE = 0.1
        test_data_length_stamps = [
            (i + 1) * TEST_LENGTH for i in range(len(TEST_SEQUENCE))
        ]
        test_data = []
        for setpoint, _ in TEST_SEQUENCE:
            test_data += [(setpoint + SCORE, setpoint + SCORE)] * TEST_LENGTH
        scores = score_speed_trajectory(test_data_length_stamps, test_data)
        print(
            f"test_data_length_stamps: {test_data_length_stamps}, test_data: {test_data}, score: {scores}"
        )
        assert np.allclose(scores, np.ones(2) * SCORE, atol=0.0001)


class TestGeneticAlgorithmPIDTuner(TestCase):
    def setUp(self) -> None:
        from tune_motor_controller import GeneticAlgorithmPIDTuner

        self._remove_performance_file()

        self.ga_pid_tuner = GeneticAlgorithmPIDTuner()

    def tearDown(self) -> None:
        self._remove_performance_file()

    def _remove_performance_file(self):
        from tune_motor_controller import LEFT_PERFORMANCE_FILE, RIGHT_PERFORMANCE_FILE

        for f in (LEFT_PERFORMANCE_FILE, RIGHT_PERFORMANCE_FILE):
            try:
                os.remove(f)
            except FileNotFoundError:
                pass

    def test_record_and_read_performance(self):
        from dream_mobile_platform.motor_controller import PIDParams

        test_ls = [
            [0.1, PIDParams(0.1, 0.1, 0.1), [0.1, 0.1, 0.1, 0.1]],
            [0.2, PIDParams(0.2, 0.2, 0.2), [0.2, 0.2, 0.2, 0.2]],
        ]

        for score, pid, single_test_data in test_ls:
            scores = (score, score)
            pid_child = (pid, pid)
            test_data = [(v, v) for v in single_test_data]
            self.ga_pid_tuner.record_score_and_update_population(
                scores, pid_child, test_data
            )


        from tune_motor_controller import LEFT_PERFORMANCE_FILE, RIGHT_PERFORMANCE_FILE
        performances = self.ga_pid_tuner._read_single_performance_file(RIGHT_PERFORMANCE_FILE)
        print("performances: ", performances)
        assert performances == test_ls
