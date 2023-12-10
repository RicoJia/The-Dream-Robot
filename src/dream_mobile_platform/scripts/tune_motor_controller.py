#!/usr/bin/env python3
"""
1. launch a process for a chosen set of PID values
2. Scoring system:
    reaching 0.1m/s, read motor speed for 5s. score: sum|score|
3. Record (kp, ki, kd): score in a csv file as a 'database'. So later it can be read
How to run this
1. start container
2. sudo_ros_preserve_env rosrun dream_mobile_platform tune_motor_controller.py <--try_best>
"""

from collections import deque
import time
import rospy
import typing
import random
import numpy as np
from multiprocessing import Process, Manager
import csv
import os

from dream_mobile_platform.motor_controller import (
    MotorControlBench,
    PIDParams,
    MotorOutputRecorder,
)
from dream_mobile_platform.motor_driver import MAX_PWM
from simple_robotics_python_utils.pubsub.shared_memory_pub_sub import SharedMemoryPub

from simple_robotics_python_utils.controllers.pid_controllers import (
    IncrementalPIDController,
    RegularDiscretePIDController,
    FeedforwardIncrementalPIDController,

)
from simple_robotics_python_utils.common.io import try_remove_file


# Fittest population should always be smaller than CHILDREN_NUM
NUM_GENERATIONS = 2
FITTEST_POPULATION_SIZE = 4
CHILDREN_NUM = int(FITTEST_POPULATION_SIZE * (FITTEST_POPULATION_SIZE - 1) / 2)
ABSOLUTE_DIR = os.path.dirname(os.path.abspath(__file__))
LEFT_PERFORMANCE_FILE = os.path.join(
    ABSOLUTE_DIR,
    "test_data",
    "LEFT_PID_PERFORMANCE.csv",
)
RIGHT_PERFORMANCE_FILE = os.path.join(
    ABSOLUTE_DIR,
    "test_data",
    "RIGHT_PID_PERFORMANCE.csv",
)

KP_MIN = 0.0
KP_MAX = 0.5
KI_MIN = 0.0
KI_MAX = 0.5
KD_MIN = 0.0
KD_MAX = 0.5
TEST_TIME = 1.5
TEST_SEQUENCE = (
    # (0.1, TEST_TIME),
    (1.0, TEST_TIME),
    (0.0, TEST_TIME),
    (0.5, TEST_TIME),
    # (1.0, TEST_TIME),
    # (0.8, TEST_TIME),
    # (0.1, TEST_TIME),
)
# Feedforward constants
NUM_STABLE_FEEDFORWARD_TERMS = 5
LEFT_PWM_FILE = os.path.join(
    ABSOLUTE_DIR,
    "test_data",
    FeedforwardIncrementalPIDController.LEFT_FEEDFOWARD_TERMS_FILE,
)
RIGHT_PWM_FILE = os.path.join(
    ABSOLUTE_DIR,
    "test_data",
    FeedforwardIncrementalPIDController.RIGHT_FEEDFOWARD_TERMS_FILE,
)

CONTROLLER_TYPE_LOOKUP = {
    "incremental": IncrementalPIDController,
    "feedforward_incremental": FeedforwardIncrementalPIDController,
    "regular": RegularDiscretePIDController,
}


def record_feedforward_terms():
    """Notes:
    1. multiprocessing.Manager uses a server to hold shared objects
        - it will only take effect if you reassign. So local operations without assigning, like
        appending, won't do anything
    """

    def record_feedforward_worker(test_pwm_to_motor_speeds):
        def record_pwm_and_two_motor_speeds(
            pwm: float, motor_speeds: typing.Tuple[float, float]
        ):
            motor_speeds_ls = test_pwm_to_motor_speeds.get(pwm, [])
            motor_speeds_ls.append(motor_speeds)
            test_pwm_to_motor_speeds[pwm] = motor_speeds_ls
            # # TODO Remember to remove
            # print(f"speed: {motor_speeds}")

        mor = MotorOutputRecorder(record_func=record_pwm_and_two_motor_speeds)

        # TODO
        time.sleep(0.2)
        for pwm in np.arange(-MAX_PWM, MAX_PWM + 0.1, 0.1):
            print(f"pub pwm: {pwm}")
            mor.pub_new_pwm(pwm)
            time.sleep(1.5)

    def write_to_csv(pwm_file: str, row: typing.List[str]):
        with open(pwm_file, "a") as f:
            writer = csv.writer(f)
            writer.writerow(["pwm", "speed"])

    with Manager() as manager:
        test_pwm_to_motor_speeds = manager.dict()
        # Test data: typing.List[typing.Tuple[float, float]
        test_proc = Process(
            target=record_feedforward_worker, args=(test_pwm_to_motor_speeds,)
        )
        test_proc.start()
        test_proc.join()
        print("test_pwm", test_pwm_to_motor_speeds)
        # Save to file
        try_remove_file(LEFT_PWM_FILE)
        try_remove_file(RIGHT_PWM_FILE)
        write_to_csv(LEFT_PWM_FILE, ["pwm", "speed"])
        write_to_csv(RIGHT_PWM_FILE, ["pwm", "speed"])
        for pwm, dual_motor_speeds in test_pwm_to_motor_speeds.items():
            stable_dual_motor_speeds = dual_motor_speeds[-NUM_STABLE_FEEDFORWARD_TERMS:]
            write_to_csv(
                LEFT_PWM_FILE,
                [
                    pwm,
                    np.average(np.array([s[0] for s in stable_dual_motor_speeds])),
                ],
            )
            write_to_csv(
                RIGHT_PWM_FILE,
                [
                    pwm,
                    np.average(np.array([s[1] for s in stable_dual_motor_speeds])),
                ],
            )


#########################################################################################
# GA Tool Functions
#########################################################################################


def generate_initial_children() -> typing.List[typing.Tuple[PIDParams, PIDParams]]:
    # Generate random numbers for P, I, D
    initial_children = []
    for _ in range(CHILDREN_NUM):
        initial_children.append(
            (
                PIDParams(
                    random.uniform(KP_MIN, KP_MAX),
                    random.uniform(KI_MIN, KI_MAX),
                    random.uniform(KD_MIN, KD_MAX),
                ),
                PIDParams(
                    random.uniform(KP_MIN, KP_MAX),
                    random.uniform(KI_MIN, KI_MAX),
                    random.uniform(KD_MIN, KD_MAX),
                ),
            )
        )
    return initial_children


def select_fittest_population(
    population: typing.Dict[PIDParams, float]
) -> typing.Dict[PIDParams, float]:
    """Select lowest N children - the higher the score, the lower the stability

    Args:
        population (typing.Dict[PIDParams, float]): left or right population

    Returns:
        typing.Dict[PIDParams, float]: lowest N children
    """
    sorted_population = sorted(population.items(), key=lambda x: x[1])
    lowest_n_children = dict(sorted_population[:FITTEST_POPULATION_SIZE])
    return lowest_n_children


def reproduce(population: typing.Dict[PIDParams, str]) -> typing.Deque[PIDParams]:
    # Take average each of them, add a mutation term to it.
    # Then you get 6 new members
    population_ls = list(population.items())
    return_population = deque()
    for i, candidate in enumerate(population_ls):
        pid = candidate[0]
        for another_candidate in population_ls[i + 1 :]:
            another_pid = another_candidate[0]
            new_pid = PIDParams(
                (pid.kp + another_pid.kp + random.uniform(0, KP_MAX)) / 3,
                (pid.ki + another_pid.ki + random.uniform(0, KP_MAX)) / 3,
                (pid.kd + another_pid.kd + random.uniform(0, KP_MAX)) / 3,
            )
            return_population.append(new_pid)
    return return_population


def combine_left_and_right_candidates(
    new_left_candidates: typing.Deque[PIDParams],
    new_right_candidates: typing.Deque[PIDParams],
) -> typing.List[typing.Tuple[PIDParams, PIDParams]]:
    children = [
        (new_left_candidates[i], new_right_candidates[i]) for i in range(CHILDREN_NUM)
    ]
    return children


def score_speed_trajectory(
    test_data_length_stamps: typing.List[int],
    test_data: typing.List[typing.Tuple[float, float]],
) -> typing.Tuple[float, float]:
    # test_data_length_stamps: [10, 20], test_data = [.....]
    # for last_length_stamp to test_data_length_stamp:
    # sum(abs(setpoint - v[k]))/len(list)
    start_of_test = 0
    i = 0
    scores = np.zeros(2)
    for setpoint, time in TEST_SEQUENCE:
        end_of_test = test_data_length_stamps[i]
        abs_diffs = abs(
            np.asarray(test_data[start_of_test:end_of_test]) - np.ones(2) * setpoint
        )
        sum_diff = np.sum(abs_diffs, axis=0)
        start_of_test = end_of_test
        scores += sum_diff / len(test_data)
        i += 1
    return tuple(scores)


def start_test_and_record(
    left_and_right_pid_params: typing.Tuple[PIDParams, PIDParams],
    controller_type: type
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
        if controller_type == FeedforwardIncrementalPIDController:
            controller = controller_type(*left_and_right_pid_params, LEFT_PWM_FILE, RIGHT_PWM_FILE)
        else:
            controller = controller_type(*left_and_right_pid_params)
        mcb = MotorControlBench(controller)

        # Publishing for 2 motors
        time.sleep(0.1)
        for v_set_point, test_time in TEST_SEQUENCE:
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
        print(f"======== Start testing child: {left_and_right_pid_params} ========")
        # Test data: typing.List[typing.Tuple[float, float]
        test_proc = Process(
            target=test_worker, args=(test_data, test_data_length_stamps)
        )
        test_proc.start()
        test_proc.join()
        if rospy.is_shutdown():
            exit(1)
        scores = score_speed_trajectory(test_data_length_stamps, test_data)
        test_data_list = list(test_data)
        # TODO Remember to remove
        print(f"Scores: {scores}")
    return scores, test_data_list


#########################################################################################
# GA Main
#########################################################################################


class GeneticAlgorithmPIDTuner:
    """Please read me

    Structure:
        - child and children are left and right motors:
        - But population, performance files are left or right motor
    """

    __slots__ = (
        "left_population",
        "right_population",
        "controller_type",
        "scores",
        "commanded_wheel_vel_pub",
    )

    def __init__(self, controller_choice: str):
        self._load_population_data()
        self.controller_type = CONTROLLER_TYPE_LOOKUP[controller_choice]

    def run(self, try_best: bool):
        if try_best:
            global CHILDREN_NUM, NUM_GENERATIONS, FITTEST_POPULATION_SIZE
            NUM_GENERATIONS = 1
            FITTEST_POPULATION_SIZE = 1
            CHILDREN_NUM = 1
        for _ in range(NUM_GENERATIONS):
            if self.left_population and self.right_population:
                left_fittest_population = select_fittest_population(
                    self.left_population
                )
                right_fittest_population = select_fittest_population(
                    self.right_population
                )
                if try_best:
                    new_left_candidates = deque(left_fittest_population.keys())
                    new_right_candidates = deque(right_fittest_population.keys())
                else:
                    new_left_candidates = reproduce(left_fittest_population)
                    new_right_candidates = reproduce(right_fittest_population)
                children: typing.List[
                    typing.Tuple[PIDParams, PIDParams]
                ] = combine_left_and_right_candidates(
                    new_left_candidates,
                    new_right_candidates,
                )
            else:
                # will execute this when there's no performance files
                children: typing.List[
                    typing.Tuple[PIDParams, PIDParams]
                ] = generate_initial_children()

            for child in children:
                scores, test_data = start_test_and_record(child, self.controller_type)
                if not try_best:
                    self.record_score_and_update_population(scores, child, test_data)
                else:
                    print(test_data)

    def record_score_and_update_population(
        self,
        scores: typing.Tuple[float, float],
        pid_child: typing.Tuple[PIDParams, PIDParams],
        test_data: typing.List[typing.Tuple[float, float]],
    ):
        def _single_record_score_and_update_population(
            score: float,
            single_pid_child: PIDParams,
            single_motor_test_data: typing.List[float],
            performance_file: str,
            population: typing.Dict[PIDParams, float],
        ):
            # Separate the left and right data
            with open(performance_file, "a") as f:
                writer = csv.writer(f)
                writer.writerow([score])
                writer.writerow(
                    [single_pid_child.kp, single_pid_child.ki, single_pid_child.kd]
                )
                writer.writerow(single_motor_test_data)
            population[single_pid_child] = score

        # get left and right test data
        left_motor_test_data = []
        right_motor_test_data = []
        for l, r in test_data:
            left_motor_test_data.append(l)
            right_motor_test_data.append(r)
        _single_record_score_and_update_population(
            scores[0],
            pid_child[0],
            left_motor_test_data,
            LEFT_PERFORMANCE_FILE,
            self.left_population,
        )
        _single_record_score_and_update_population(
            scores[1],
            pid_child[1],
            right_motor_test_data,
            RIGHT_PERFORMANCE_FILE,
            self.right_population,
        )
        # record the score in LEFT_PERFORMANCE_FILE, RIGHT_PERFORMANCE_FILE

    def _read_single_performance_file(
        self, performance_file
    ) -> typing.List[typing.List[typing.Any]]:
        return_performances = []
        # pid, score, test_data are the recorded items
        NUM_RECORDED_ITEMS = 3
        try:
            with open(performance_file, "r") as f:
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
        except FileNotFoundError:
            # No performance file is found, we return empty list here
            pass
        # typing.List[(float, PIDParams, typing.List[float])]
        return return_performances

    def _load_population_data(self):
        def _load_single_population(
            population: typing.Dict[PIDParams, float], performance_file: str
        ):
            performances = self._read_single_performance_file(performance_file)
            for p in performances:
                score = p[0]
                pid = p[1]
                population[pid] = score

        self.left_population: typing.Dict[PIDParams, float] = {}
        self.right_population: typing.Dict[PIDParams, float] = {}
        _load_single_population(self.left_population, LEFT_PERFORMANCE_FILE)
        _load_single_population(self.right_population, RIGHT_PERFORMANCE_FILE)

    def summarize(self):
        def _summarize_single_performance_file(performance_file):
            # typing.List[(float, PIDParams, typing.List[float])]
            all_performances = self._read_single_performance_file(performance_file)
            print(f"============ {performance_file} ============")
            # typing.Dict[PIDParams, float]
            fittest = select_fittest_population({p[1]: p[0] for p in all_performances})
            print(fittest)

        _summarize_single_performance_file(LEFT_PERFORMANCE_FILE)
        _summarize_single_performance_file(RIGHT_PERFORMANCE_FILE)


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
        1. Initialize population of 6
        2. From the entire find the 4 fittest members; 
        3. Cross over each of them, add a mutation term to it. Then you get 6 new members
        4. Try the new 6 members. 
        5. Add the new 6 members to the population
    4. Termniate when the fittest members are the same. 
        1. print number of iteration
        2. Or when ctrl-C is hit.

3. Detailed usage: TODO: to organize
    1. GA reads the existing performance file and use them to generate subsequent seeds.
        - So delete it and start testing again.
"""
if __name__ == "__main__":
    # DO NOT LAUNCH motor_controller.py. Setting node name to test_motors
    import argparse

    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--try_best",
        action="store_true",
        help="If you have performance files already, try the best ones",
    )
    parser.add_argument(
        "--record_feedforward",
        action="store_true",
        help="If you want to test feedforward controller, you need feedforward data"
    )
    controller_choices = list(CONTROLLER_TYPE_LOOKUP.keys())
    parser.add_argument(
        "--controller",
        type=str,
        default="feedforward_incremental",
        choices=controller_choices,
        help="Which controller to use, please see choices",
    )

    args = parser.parse_args()
    rospy.init_node("test_motors")

    if args.controller == "feedforward_incremental" and args.record_feedforward:
        record_feedforward_terms()
    # TODO: this is kind of hacky, basically, now we have performance files ending in controller type
    LEFT_PERFORMANCE_FILE += f".{args.controller}"
    RIGHT_PERFORMANCE_FILE += f".{args.controller}"
    ga_pid_tuner = GeneticAlgorithmPIDTuner(args.controller)
    ga_pid_tuner.run(args.try_best)
    ga_pid_tuner.summarize()
