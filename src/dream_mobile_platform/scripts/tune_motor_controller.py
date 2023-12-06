#!/usr/bin/env python3
"""
1. launch a process for a chosen set of PID values
2. Scoring system:
    reaching 0.1m/s, read motor speed for 5s. score: sum|score|
3. Record (kp, ki, kd): score in a csv file as a 'database'. So later it can be read
How to run this
1. start container
2. sudo_ros_preserve_env rosrun dream_mobile_platform tune_motor_controller.py
"""

from collections import deque
import time
import rospy
import typing
import random
import numpy as np
from multiprocessing import Process, Manager
import csv

from dream_mobile_platform.motor_controller import MotorControlBench, PIDParams
from simple_robotics_python_utils.pubsub.shared_memory_pub_sub import SharedMemoryPub


FITTEST_POPULATION_SIZE = 10
LEFT_PERFORMANCE_FILE = "LEFT_PID_PERFORMANCE.csv"
RIGHT_PERFORMANCE_FILE = "RIGHT_PID_PERFORMANCE.csv"
KP_MAX = 2
KI_MAX = 0.3
KD_MAX = 0.3
# TODO: before we run this, we need to see if these velocities make sense
TEST_SEQUENCE = (
    (0.1, 1.5),
    # (0.6, 1.5),
    # (0.2, 1.5),
)  
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
    # TODO
    pass


def reproduce(parents):
    # cross over, then mutate
    # TODO
    pass


def score_speed_trajectory(test_data_length_stamps, test_data):
    # test_data_length_stamps: [10, 20], test_data = [.....]
    # for last_length_stamp to test_data_length_stamp:
    # sum(abs(setpoint - v[k]))/len(list)
    start_of_test = 0
    i = 0
    score = 0.0
    for setpoint, time in TEST_SEQUENCE:
        end_of_test = test_data_length_stamps[i]
        sum_diff = sum(
            abs(
                np.asarray(test_data[start_of_test:end_of_test]) - np.ones(2) * setpoint
            )
        )
        start_of_test = end_of_test
        score += sum_diff / len(test_data)
        i += 1
    return score


def start_test_and_record(
    left_and_right_pid_params: typing.Tuple[PIDParams, PIDParams]
) -> typing.Tuple[float, typing.List[float]]:
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
            # TODO
            # commanded_wheel_vel_pub.publish([v_set_point, v_set_point])
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
        # Test data: typing.List[typing.Tuple[float, float]
        test_proc = Process(
            target=test_worker, args=(test_data, test_data_length_stamps)
        )
        test_proc.start()
        test_proc.join()
        # TODO Remember to remove
        print(f"Rico: test data: {test_data}")
        score = score_speed_trajectory(test_data_length_stamps, test_data)
        test_data_list = list(test_data)
    return score, test_data_list


class GeneticAlgorithmPIDTuner:
    __slots__ = ("left_population", "right_population", "scores", "commanded_wheel_vel_pub")

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
                score, test_data = start_test_and_record(child)
                self.record_score_and_update_population(score, child, test_data)

    def _single_record_score_and_update_population(
        score: float, 
        single_pid_child: PIDParams
        single_motor_test_data: typing.List[float],
        PERFORMANCE_FILE
        ):
        # Separate the left and right data
        with open(PERFORMANCE_FILE, "a") as f:
            writer = csv.writer(f)
            writer.writerow([score])
            writer.writerow([pid.kp, pid.ki, pid.kd])
            writer.writerow(test_data)
        self.population[pid] = score
    
    def record_score_and_update_population(
        self, 
        score: float, 
        pid_child: typing.Tuple[PIDParams, PIDParams], 
        test_data: typing.List[typing.Tuple[float, float]]
    ):
        # record the score in LEFT_PERFORMANCE_FILE, RIGHT_PERFORMANCE_FILE 

    def read_performance_file(self) -> typing.List[typing.List[typing.Any]]:
        return_performances = []
        # pid, score, test_data are the recorded items
        NUM_RECORDED_ITEMS = 3
        with open(PERFORMANCE_FILE, "r") as f:
            reader = csv.reader(f)
            for row_i, row in enumerate(reader):
                if row_i % NUM_RECORDED_ITEMS == 0:
                    return_performances.append([float(row[0])])
                elif row_i % NUM_RECORDED_ITEMS == 1:
                    return_performances[-1].append(
                        PIDParams(float(row[0]), float(row[1]), float(row[2]))
                    )
                elif row_i % NUM_RECORDED_ITEMS == 2:
                    return_performances[-1].append([float(x) for x in row])
        # typing.List[(float, PIDParams, typing.List[float])]
        return return_performances

    def _load_population_data(self):
        # TODO
        self.population = {}


"""
1. Design decision on lanching a separate process for a single test bench
    1. In an ideal world, we can have a publisher and a subscriber automatically recycled, 
        - That requires: 1. when recycled, it will say bye to its peers. 2. force gc
    2. Intermediate solution: The instances still exists, but they are unregistered
    3. Or, We can share pub, and sub. Make them singleton. However, Subscribers needs callback. 
        - It's not a good practice to change subscriber callback
    4. Or, we launch a separate process for the test bench. 
        - Need: two lists: test data, and timestamp. And that requires multiprocessing.manager, and sharedlist. 
        - Implementation is not too complicated, and it's forward compatible.

2. How to tune: Let's try genetic algorithm with incremental PID first. Then, try feedforward if not  good
    1. Implement scoring
    2. Store scores in performance. 
    3. Implement GA with: 
        1. initialize population of 6
        2. From the entire find the 4 fittest members; Cross over each of them, add a mutation term to it. Then you get 6 new members
        3. Try the new 6 members. 
        4. Add the new 6 members to the population
    4. Termniate when the fittest members are the same. 
        1. print number of iteration
        2. Or when ctrl-C is hit.

"""
if __name__ == "__main__":
    # DO NOT LAUNCH motor_controller.py. Setting node name to test_motors
    rospy.init_node("test_motors")
    ga_pid_tuner = GeneticAlgorithmPIDTuner()
    ga_pid_tuner.run()
