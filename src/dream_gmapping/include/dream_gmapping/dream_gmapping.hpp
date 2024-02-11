#pragma once
#include "dream_gmapping/particle_filter.hpp"
#include <geometry_msgs/TransformStamped.h>
#include <memory>
#include <message_filters/subscriber.h>
#include <ros/node_handle.h>
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <tf2_ros/message_filter.h>
#include <tf2_ros/transform_listener.h>
#include <vector>

namespace DreamGMapping {
class DreamGMapper {
public:
  DreamGMapper(ros::NodeHandle &nh_);
  ~DreamGMapper();
  //
  /** \brief Main function for evaluating particles and generating maps
      this signature is required for scan_filter_ptr_
  */
  void
  laser_scan(const boost::shared_ptr<const sensor_msgs::LaserScan> &scan_msg);

protected:
  // configurable parameters
  std::string base_frame_ = "base_link";
  std::string map_frame_ = "map";
  std::string odom_frame_ = "odom";
  double map_update_interval_ = 0.1; // 10 hz
  double max_range_;
  int particle_num_ = 50;

  // inconfigurable parameters
  bool received_first_laser_scan_ = false;
  tf2_ros::Buffer tf_buffer_ = tf2_ros::Buffer();
  tf2_ros::TransformListener tf_listener_ =
      tf2_ros::TransformListener(tf_buffer_);
  std::unique_ptr<message_filters::Subscriber<sensor_msgs::LaserScan>>
      scan_sub_ptr_;
  std::unique_ptr<tf2_ros::MessageFilter<sensor_msgs::LaserScan>>
      scan_filter_ptr_;

  std::vector<DreamGMapping::Particle> particles_;
};
} // namespace DreamGMapping