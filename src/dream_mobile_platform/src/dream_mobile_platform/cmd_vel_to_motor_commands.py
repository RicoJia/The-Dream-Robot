#!/usr/bin/env python3

"""
How to run:
sudo_ros_preserve_env rosrun dream_mobile_platform cmd_vel_to_motor_commands.py
"""
import rospy
from geometry_msgs.msg import Twist
from simple_robotics_python_utils.pubsub.shared_memory_pub_sub import SharedMemoryPub


class CmdvelToMotorCommands:
    def __init__(self, wheel_base):
        # Distance between the wheels and radius of the wheels
        self.wheel_base = wheel_base

        # Subscribers and Publishers
        # This is where the rostopic and shared pub meets
        self.cmd_vel_sub = rospy.Subscriber(
            rospy.get_param("/ROS_TOPIC/CMD_VEL"),
            Twist,
            self.cmd_vel_callback,
        )
        self.wheel_vel_pub = SharedMemoryPub(
            topic=rospy.get_param("/SHM_TOPIC/COMMANDED_WHEEL_VELOCITY"),
            data_type=float,
            arr_size=2,
            debug=False,
        )

    def cmd_vel_callback(self, msg):
        # msg should be in meters and rad/s
        linear_vel = msg.linear.x
        angular_vel = msg.angular.z
        # Compute velocities for each wheel
        left_wheel_vel = linear_vel - (self.wheel_base / 2.0) * angular_vel
        # Note, we want to negate right_wheel_vel, because its positive direction
        # on the robot makes it go backwards
        right_wheel_vel = -(linear_vel + (self.wheel_base / 2.0) * angular_vel)

        # Publish wheel velocities in m/s
        self.wheel_vel_pub.publish([left_wheel_vel, right_wheel_vel])
        
        #TODO Remember to remove
        # print(f'Rico: received {msg}')


if __name__ == "__main__":
    rospy.init_node("cmd_vel_to_motor_commands")
    # Distance between wheels centers (in meters)
    wheel_base = rospy.get_param("/PARAMS/WHEEL_BASE_CENTER_TO_CENTER")
    converter = CmdvelToMotorCommands(wheel_base)
    rospy.spin()
