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
#include <std_msgs/Float32MultiArray.h>

namespace RosUtils {
    inline void print_all_nodehandle_params(ros::NodeHandle nh){
        std::vector<std::string> params;
        nh.getParamNames(params);
        for (auto &param : params) {
            std::string param_val;
            nh.getParam(param, param_val);
            ROS_INFO_STREAM("param: " << param << " val: " << param_val);
        }
    }
}

namespace DreamGMapping {
class DreamGMapper {
public:
  explicit DreamGMapper(ros::NodeHandle nh_);
  ~DreamGMapper();
  //
  /** \brief Main function for evaluating particles and generating maps
      this signature is required for scan_filter_ptr_
  */
  void laser_scan(const boost::shared_ptr<const sensor_msgs::LaserScan> &scan_msg);
  void wheel_odom(const std_msgs::Float32MultiArray::ConstPtr &odom_msg);

protected:
  // configurable parameters
  std::string base_frame_ = "base_link";
  std::string map_frame_ = "map";
  std::string odom_frame_ = "odom";
  double map_update_interval_ = 0.1; // 10 hz
  double max_range_;
  int particle_num_ = 50;
  Eigen::Matrix3d motion_covariances_;
  Eigen::Vector3d motion_means_;

  // inconfigurable parameters
  // no need to store 
  bool received_first_laser_scan_ = false;
  ros::Subscriber laser_sub_;
  tf2_ros::Buffer tf_buffer_ = tf2_ros::Buffer();
  tf2_ros::TransformListener tf_listener_ =
      tf2_ros::TransformListener(tf_buffer_);

  ros::Subscriber wheel_odom_sub_;
  std::pair<double, double> last_wheel_odom_;
  std::pair<double, double> current_wheel_odom_;

  std::vector<float> last_scan_;
  std::vector<DreamGMapping::Particle> particles_;

  // get the most recent odom -> draw a new noise -> go through all particles, 
  void store_last_scan(const boost::shared_ptr<const sensor_msgs::LaserScan> &scan_msg);
  void update_with_motion_model();
  void scan_match_for_guess();

};
} // namespace DreamGMapping