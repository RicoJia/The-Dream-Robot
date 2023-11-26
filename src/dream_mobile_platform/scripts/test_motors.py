#!/usr/bin/env python3
"""
1. launch a process for a chosen set of PID values
2. Scoring system:
    reaching 0.1m/s, read motor speed for 5s. score: sum|score|
3. Record (kp, ki, kd): score in a csv file as a 'database'. So later it can be read
"""

from collections import deque
import time
from dream_mobile_platform.motor_controller import MotorControlBench, PIDParams
from simple_robotics_python_utils.pubsub.shared_memory_pub_sub import SharedMemoryPub
import rospy
import typing
import random
from multiprocessing import Process, Manager

FITTEST_POPULATION_SIZE = 10
PERFORMANCE_FILE = "PID_PERFORMANCE.csv"
KP_MAX = 1
KI_MAX = 0.1
KD_MAX = 0.3
# TODO: before we run this, we need to see if these velocities make sense
TEST_SEQUENCE = (
    (0.1, 5),
    (0.6, 5),
)  # (0.2, 5), (0.3, 5), (0.1, 5), (0.0, 5)
NUM_GENERATIONS = 1
CHILDREN_NUM = 1


def generate_initial_children() -> typing.List[typing.Tuple[PIDParams, PIDParams]]:
    # Generate random numbers for P, I, D
    initial_children = []
    for _ in range(CHILDREN_NUM):
        initial_children.append(
            (
                PIDParams(
                    random.uniform(0, KP_MAX),
                    random.uniform(0, KI_MAX),
                    random.uniform(0, KD_MAX),
                ),
                PIDParams(
                    random.uniform(0, KP_MAX),
                    random.uniform(0, KI_MAX),
                    random.uniform(0, KD_MAX),
                ),
            )
        )
    return initial_children


def select_fittest_population(population):
    pass


def reproduce(parents):
    # cross over, then mutate
    pass


def score_speed_trajectory(test_data_length_stamps, test_data):
    pass


def start_test_and_record(
    left_and_right_pid_params: typing.Tuple[PIDParams, PIDParams]
):
    """
    - test_output = []; will be nice to have (0.5 - [])...
    - controller has a subscriber -> encoder; sub -> commanded wheel vel; publisher -> speed pub;
        - ideal case: when timeout -> records length of output list -> keep stepping end of the test: sum(abs(setpoint - v[k]))/len(list)
        - currently:
            1. controller automatically pubs 0 if the subscribers are down.
            2. Otherwise, it will publish velocity explictly.
    - Running Env: dream_byobu without motor_controller launched.
    """

    def test_worker(test_data, test_data_length_stamps):
        # launching a pub because it has to be within the same process
        commanded_wheel_vel_pub = SharedMemoryPub(
            topic=rospy.get_param("/SHM_TOPIC/COMMANDED_WHEEL_VELOCITY"),
            data_type=float,
            arr_size=2,
            debug=False,
        )
        mcb = MotorControlBench(*left_and_right_pid_params)
        # Publishing for 2 motors
        time.sleep(0.1)
        for v_set_point, test_time in TEST_SEQUENCE:
            # TODO Remember to remove
            print(f"Rico: set_point: {v_set_point}")
            start_time = time.perf_counter()
            commanded_wheel_vel_pub.publish([v_set_point, v_set_point])
            while time.perf_counter() - start_time < test_time:
                # place the speed reading before stepping, because it takes
                # time for the step to take effect.
                actual_speeds = mcb.get_actual_speeds()
                mcb.step()
                test_data.append(actual_speeds)
            test_data_length_stamps.append(len(test_data))
            commanded_wheel_vel_pub.publish([0.0, 0.0])

    with Manager() as manager:
        test_data = manager.list()
        test_data_length_stamps = manager.list()
        print(f"Start testing child: {left_and_right_pid_params}")
        test_proc = Process(
            target=test_worker, args=(test_data, test_data_length_stamps)
        )
        test_proc.start()
        test_proc.join()
        # TODO Remember to remove
        print(f"Rico: test data: {test_data}")
        score = score_speed_trajectory(test_data_length_stamps, test_data)
    return score


class GeneticAlgorithmPIDTuner:
    __slots__ = ("population", "scores", "commanded_wheel_vel_pub")

    def __init__(self):
        self._load_population_data()

    def run(self):
        for _ in range(NUM_GENERATIONS):
            if self.population:
                fittest_population = select_fittest_population(self.population)
                children: typing.List[typing.Tuple[PIDParams, PIDParams]] = reproduce(
                    fittest_population
                )
            else:
                children: typing.List[
                    typing.Tuple[PIDParams, PIDParams]
                ] = generate_initial_children()
            for child in children:
                score = start_test_and_record(child)
                self.record_score_and_update_population(score, child)

    def record_score_and_update_population(self, score, pid):
        # record the score in PERFORMANCE_FILE
        # add the child and score to population
        pass

    def _load_population_data(self):
        # TODO : to load params properly
        self.population = {}


"""
Design decision on lanching a separate process for a single test bench
    1. In an ideal world, we can have a publisher and a subscriber automatically recycled, 
        - That requires: 1. when recycled, it will say bye to its peers. 2. force gc
    2. Intermediate solution: The instances still exists, but they are unregistered
    3. Or, We can share pub, and sub. Make them singleton. However, Subscribers needs callback. 
        - It's not a good practice to change subscriber callback
    4. Or, we launch a separate process for the test bench. 
        - Need: two lists: test data, and timestamp. And that requires multiprocessing.manager, and sharedlist. 
        - Implementation is not too complicated, and it's forward compatible.
"""
if __name__ == "__main__":
    # DO NOT LAUNCH motor_controller.py. Setting node name to test_motors
    rospy.init_node("test_motors")
    ga_pid_tuner = GeneticAlgorithmPIDTuner()
    ga_pid_tuner.run()
