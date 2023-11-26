"""
1. launch a process for a chosen set of PID values
2. Scoring system:
    reaching 0.1m/s, read motor speed for 5s. score: sum|score|
3. Record (kp, ki, kd): score in a csv file as a 'database'. So later it can be read
"""

from collections import deque
import time
from dream_mobile_platform.motor_controller import MotorControlBench

NUM_RUNS = 100
FITTEST_POPULATION_SIZE = 10
CHILDREN_NUM = 5
# TODO: before we run this, we need to see if these velocities make sense
TEST_SEQUENCE = ((0.1, 5), (0.2, 5), (0.3, 5), (0.1, 5), (0.0, 5))
PERFORMANCE_FILE = "PID_PERFORMANCE.csv"

def select_fittest_population(population):
    pass

def reproduce(parents):
    # cross over, then mutate
    pass

def score_speed_trajectory(test_data_length_stamps, test_data):
    pass

def start_test_and_record(pid):
    """
    - test_output = []; will be nice to have (0.5 - [])...
    - controller has a subscriber -> encoder; sub -> commanded wheel vel; publisher -> speed pub; 
        - ideal case: when timeout -> records length of output list -> keep stepping end of the test: sum(abs(setpoint - v[k]))/len(list)
        - currently: 
            1. controller automatically pubs 0 if the subscribers are down. 
            2. Otherwise, it will publish velocity explictly.
    - Running Env: dream_byobu without motor_controller launched.
    """
    test_data = deque()
    test_data_length_stamps = []
    for v_set_point, test_time in TEST_SEQUENCE:
        mcb = MotorControlBench(pid)
        start_time = time.perf_counter()
        while time.perf_counter() - start_time < test_time:
            mcb.step()
        test_data_length_stamps.append(len(test_data))
    score = score_speed_trajectory(test_data_length_stamps, test_data) 
    return score
        
        
class GeneticAlgorithmPIDTuner:
    __slots__ = ('population', 'scores')
    def __init__(self):
        self._load_population_data()
        self._generate_initial_population()

    def run(self):
        while _ in range(NUM_RUNS):
           fittest_population = select_fittest_population(self.population)
           children = reproduce(fittest_population)
           for child in children:
               score = start_test_and_record(child)
               self.record_score_and_update_population(score, child)

    def record_score_and_update_population(self, score, pid):
        # record the score in PERFORMANCE_FILE
        # add the child and score to population
        pass
        
    def _load_population_data(self):
        pass
    
    def _generate_initial_population(self):
        pass
