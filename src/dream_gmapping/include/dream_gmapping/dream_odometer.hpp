#pragma once

#include "simple_robotics_cpp_utils/math_utils.hpp"
#include "simple_robotics_cpp_utils/rigid2d.hpp"
#include <geometry_msgs/TransformStamped.h>
#include <ros/ros.h>
#include <std_msgs/Float32MultiArray.h>
#include <tf2_eigen/tf2_eigen.h>
#include <tf2_ros/transform_broadcaster.h>

using SimpleRoboticsCppUtils::find_smaller_delta;
using SimpleRoboticsCppUtils::get_2D_screw_displacement;
using SimpleRoboticsCppUtils::normalize_angle_2PI;
using SimpleRoboticsCppUtils::screw_displacement_2d_to_body_frame_transform;

namespace DreamGMapping {
class DreamOdometer {
protected:
  ros::Subscriber wheel_odom_sub_;
  std::pair<double, double> last_wheel_positions_{0.0, 0.0};
  Eigen::Matrix4d tf_matrix_ = Eigen::Matrix4d::Identity();
  double wheel_dist_;
  double wheel_radius_;
  std::string base_frame_ = "base_link";
  std::string odom_frame_ = "odom";
  geometry_msgs::TransformStamped tf_msg_;
  tf2_ros::TransformBroadcaster br_;

  // [left, right], the wheel positions are [0, 2pi]
  void wheel_odom(const std_msgs::Float32MultiArray::ConstPtr &odom_msg) {
    auto screw_displacement = get_2D_screw_displacement(
        {find_smaller_delta(last_wheel_positions_.first, odom_msg->data[0]) *
             wheel_radius_,
         // flipping right wheel because 2D screw displacement function assumes
         // CW as positive
         -find_smaller_delta(last_wheel_positions_.second, odom_msg->data[1]) *
             wheel_radius_},
        wheel_dist_);
    auto body_frame_tf =
        screw_displacement_2d_to_body_frame_transform(screw_displacement);
    tf_matrix_ = tf_matrix_ * body_frame_tf;
    tf_msg_.header.stamp = ros::Time::now();
    tf_msg_.transform =
        tf2::eigenToTransform(Eigen::Affine3d(tf_matrix_)).transform;
    br_.sendTransform(tf_msg_);
    // flipping right wheel because 2D screw displacement function assumes CW as
    // positive
    last_wheel_positions_ = {odom_msg->data[0], odom_msg->data[1]};
  }

public:
  DreamOdometer(ros::NodeHandle nh_) {
    std::string wheel_pos_topic;
    nh_.getParam("wheel_pos_topic", wheel_pos_topic);
    wheel_odom_sub_ =
        nh_.subscribe(wheel_pos_topic, 1, &DreamOdometer::wheel_odom, this);
    nh_.getParam("wheel_dist", wheel_dist_);
    nh_.getParam("wheel_diameter", wheel_radius_);
    wheel_radius_ /= 2.0;
    nh_.getParam("base_frame", base_frame_);
    nh_.getParam("odom_frame", odom_frame_);

    ROS_INFO_STREAM("Odometer is subscribing to " << wheel_pos_topic);

    tf_msg_.header.frame_id = odom_frame_;
    tf_msg_.child_frame_id = base_frame_;
  }
};

} // namespace DreamGMapping