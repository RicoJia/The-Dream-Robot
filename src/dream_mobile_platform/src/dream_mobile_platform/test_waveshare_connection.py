import serial
import struct
import json
import time
import threading
from collections import deque
from dataclasses import dataclass

# ================================================
# Configuration and Constants
# ================================================

# Rover connection settings
ROVER_SERIAL_PORT = '/dev/serial0'
ROVER_BAUD_RATE = 115200

COMMANDS = {
    "FORWARD": {"T": 1, "L": 150, "R": 150},  # Normal forward, adjust speed as needed
    "REVERSE": {"T": 1, "L": -200, "R": -200},  # Fast reverse, adjust speed as needed
    "TURN_LEFT": {"T": 1, "L": -0.5, "R": 0.5},  # Sharp left turn, negative value for left motor, positive for right
    "TURN_RIGHT": {"T": 1, "L": 255, "R": -255},  # Sharp right turn, positive value for left motor, negative for right
    "STOP": {"T": 0},  # Emergency stop
    "OLED": {"T":3,"lineNum":0,"Text":"putYourTextHere"},
    "RESTORE_OLED": {"T":-3},
    "READ_IMU": {"T":126},
}


def open_serial_connection(port, baud_rate):
    """Attempts to open a serial connection to a given port with specified baud rate."""
    try:
        return serial.Serial(port, baud_rate, timeout=1)
    except serial.SerialException as e:
        print(f"Failed to open serial port {port}: {e}")
        return None

def read_response_from_rover(serial_conn, timeout=2):
    """
    Reads a JSON-formatted response from the rover over a serial connection.
    
    Args:
        serial_conn (serial.Serial): The serial connection to the rover.
        timeout (int, optional): Time in seconds to wait for a response. Defaults to 2.
    
    Returns:
        dict or None: The parsed JSON response as a dictionary, or None if failed.
    """
    serial_conn.timeout = timeout  # Set read timeout
    try:
        response = serial_conn.readline().decode().strip()  # Read a line and decode
        if response:
            return json.loads(response)  # Parse JSON
        else:
            return None
    except json.JSONDecodeError as e:
        print(f"Failed to parse JSON response: {e}")
        return None
    except serial.SerialException as e:
        print(f"Failed to read response from rover: {e}")
        return None

def send_command_to_rover(serial_conn, command):
    """Sends a JSON-formatted command to the rover over a serial connection."""
    command_str = json.dumps(command) + '\n'
    try:
        serial_conn.write(command_str.encode())
    except serial.SerialException as e:
        print(f"Failed to send command to rover: {e}")

def test_imu_reading(rover_conn):
    while True:
        start = time.time()
        send_command_to_rover(rover_conn, COMMANDS["READ_IMU"])
        res = read_response_from_rover(rover_conn)
        end = time.time()
        print(f'Rico: ======================')
        print(f'elapsed {end - start}')
        print(f'res: {res}')

@dataclass
class IMUReading:
    # TODO: more?
    timestamp: int
    ax: float = 0.0
    ay: float = 0.0
    az: float = 0.0
    def __add__(self, other):
        return IMUReading(
            ax=self.ax + other.ax,
            ay=self.ay + other.ay,
            az=self.az + other.az,
            timestamp=max(self.timestamp, other.timestamp)
        )
        
    def __truediv__(self, scalar):
        return IMUReading(
            ax = self.ax / scalar,
            ay = self.ay / scalar,
            az = self.az / scalar,
            timestamp = self.timestamp
        )
    
# TODO: for simple robotics

class Timer:
    """
    A python object that spawns a daemon thread for callback firing. This should always be spawned later
    """
    def __init__(self, frequency, cb, args=[], kwargs={}):
        # Start the timer thread
        self.interval= 1.0/frequency
        self.cb = cb
        self.args = args
        self.kwargs = kwargs
        self.timer_thread = threading.Thread(target=self.timer_function, daemon=True)
        self.timer_thread.start()
    def timer_function(self):
        while True:
            start_time = time.time()
            self.cb(*self.args, **self.kwargs)
            elapsed_time = time.time() - start_time
            sleep_time = max(0, self.interval - elapsed_time)
            time.sleep(sleep_time)
        

class IMUFilter:
    def __init__(self, rover_conn, frequency, buffer_time, imu_single_query_frequency = 20, callbacks = []):
        self.callbacks = callbacks
        self.rover_conn = rover_conn
        self.buffer_time = buffer_time
        # TODO: A timer thread?
        self.dq = deque(maxlen=int(buffer_time * imu_single_query_frequency))
        self.lock = threading.Lock()  # For thread-safe operations on the deque
        self.query_timer = Timer(frequency=imu_single_query_frequency, cb = self.query_imu)
        self.smoothing_timer = Timer(frequency=frequency, cb = self.timer_function)
        
    def query_imu(self):
        send_command_to_rover(self.rover_conn, COMMANDS["READ_IMU"])
        res = read_response_from_rover(self.rover_conn)
        # res: {'T': 1002, 'r': -1.537751794, 'p': -16.31915665, 'y': 122.3150177, 'ax': 315.6152344, 'ay': -39.921875, 'az': 1074.394531, 'gx': 10.49750042, 'gy': 8.511249542, 'gz': 8.668749809, 'mx': 23, 'my': 93, 'mz': -94, 'temp': 0}
        if res['T'] == 1002:
            r = IMUReading(
                ax = res.get("ax", 0.0),
                ay = res.get("ay", 0.0),
                az = res.get("az", 0.0),
                timestamp = time.time()
                # TODO: more?
            )
            with self.lock:
                self.dq.append(r)

    def timer_function(self):
        """
        - the raw output is in milli-g. To get the accerlation in m/s2, one needs to do 0.00980665 * val
        """
        current_time = time.perf_counter()
        earliest_time = current_time - self.buffer_time
        # # Apply a time window for smoothing here
        speed = IMUReading(timestamp=earliest_time)
        total_readings = 0
        with self.lock:
            # There should be at least 1 item in dq
            while self.dq and self.dq[0].timestamp < earliest_time:
                self.dq.popleft()
            if not self.dq:
                return
            for reading in self.dq:
                speed += reading
                total_readings += 1
        average_speed = speed/total_readings
        print("average_speed: ", average_speed)
        print("speed: ", speed)
        # for cb in self.callbacks:
        #     cb(speed)

def test_imu_smoothing(rover_conn):
    filter = IMUFilter(
        rover_conn, frequency = 10, buffer_time = 0.2, imu_single_query_frequency = 20, callbacks = []
    )
    time.sleep(10)

def main():
    """Main function to initialize connections and control the rover based on LiDAR data."""
    rover_conn = open_serial_connection(ROVER_SERIAL_PORT, ROVER_BAUD_RATE)
    if not rover_conn:
        print("Unable to open serial connections. Exiting.")
        return
    # send_command_to_rover(rover_conn, COMMANDS["RESTORE_OLED"])
    # send_command_to_rover(rover_conn, COMMANDS["TURN_LEFT"])
    # time.sleep(0.5)
    # send_command_to_rover(rover_conn, COMMANDS["TURN_RIGHT"])
    # time.sleep(0.5)
    # send_command_to_rover(rover_conn, COMMANDS["STOP"])
    # test_imu_reading(rover_conn)
    test_imu_smoothing(rover_conn)

main()
