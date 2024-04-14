#!/usr/bin/env python
import rospy
import numpy as np
import pickle
from sensor_msgs.msg import LaserScan

scan_data_list = []  # List to hold multiple scan data arrays

def scan_callback(msg):
    # Convert the LaserScan message to a NumPy array
    ranges = np.array(msg.ranges)
    scan_data_list.append(ranges)  # Append to the list
    rospy.loginfo("Scan data appended to list")

    # Limit the size of the list if necessary (e.g., to the last 10 scans)
    # scan_data_list = scan_data_list[-10:]

def listener():
    rospy.init_node('scan_saver')
    rospy.Subscriber('/dream/scan', LaserScan, scan_callback)
    rospy.spin()

    # Once rospy.spin() is complete (e.g., the node is shutting down), save the data
    with open('scan_data.pkl', 'wb') as file:
        pickle.dump(scan_data_list, file)

if __name__ == '__main__':
    listener()