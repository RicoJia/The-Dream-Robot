#!/usr/bin/env python3
from simple_robotics_python_utils.sensors.rpi_lidar_a1_visualization import find_laser_scanner_rpi, TOTAL_NUM_ANGLES
from adafruit_rplidar import RPLidar
import rospy
from math import floor
from typing import List
from sensor_msgs.msg import LaserScan

def get_initialized_laser_scan_msg():
    msg = LaserScan()
    msg.header.frame_id = rospy.get_param('/PARAMS/SCAN_FRAME')
    msg.angle_min = 0.0
    msg.angle_max = 6.28
    msg.angle_increment = (msg.angle_max - msg.angle_min) / TOTAL_NUM_ANGLES
    msg.range_min = 0.0
    msg.range_max = 12.0
    return msg

def publish_msg(scan_data: List[float], msg:LaserScan, publisher: rospy.Publisher):
    msg.header.stamp = rospy.Time.now()
    msg.ranges = scan_data
    publisher.publish(msg)

if __name__ == '__main__':
    NODE_NAME = 'rpi_lidar_a1_node'
    device_address = find_laser_scanner_rpi()
    rospy.init_node(NODE_NAME)
    lidar = RPLidar(None, device_address)

    msg = get_initialized_laser_scan_msg()
    publisher = rospy.Publisher(
        rospy.get_param('/ROS_TOPIC/SCAN_TOPIC'), LaserScan, queue_size=10
    )
    rospy.loginfo(f"{NODE_NAME}: initialized")
    while not rospy.is_shutdown():
        try:
            scan_data = [0] * TOTAL_NUM_ANGLES
            for scan in lidar.iter_scans():
                for _, angle, distance in scan:
                    # preprocessing
                    # raw output is in miliseconds
                    scan_data[min(359, floor(angle))] = distance/1000.0
                publish_msg(scan_data, msg, publisher)
        except Exception:
            pass
    lidar.stop()
    lidar.disconnect()
        