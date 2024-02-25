#pragma once
#include "dream_gmapping/dream_gmapping_utils.hpp"
#include <cmath>
#include <geometry_msgs/TransformStamped.h>
#include <memory>
#include <message_filters/subscriber.h>
#include <ros/node_handle.h>
#include <ros/ros.h>
#include <simple_robotics_cpp_utils/math_utils.hpp>
#include <simple_robotics_cpp_utils/rigid2d.hpp>
#include <std_msgs/Float32MultiArray.h>
#include <tf2_ros/message_filter.h>
#include <tf2_ros/transform_listener.h>
#include <vector>

namespace DreamGMapping {
class DreamGMapper {
public:
  explicit DreamGMapper(ros::NodeHandle nh_);
  ~DreamGMapper();
  //
  /** \brief Main function for evaluating particles and generating maps
      this signature is required for scan_filter_ptr_
  */
  void
  laser_scan(const boost::shared_ptr<const sensor_msgs::LaserScan> &scan_msg);
  void wheel_odom(const std_msgs::Float32MultiArray::ConstPtr &odom_msg);

protected:
  // configurable parameters
  std::string base_frame_ = "base_link";
  std::string map_frame_ = "map";
  std::string odom_frame_ = "odom";
  double map_update_interval_ = 0.1; // 10 hz
  double max_range_;
  double wheel_dist_;
  int particle_num_ = 50;
  double angular_active_threshold_ = M_PI / 6.0;
  double translation_active_threshold_ = 0.2;
  Eigen::Matrix2d motion_covariances_{Eigen::Matrix2d::Identity()};
  // No need to be user-initialized
  Eigen::Vector3d motion_means_{Eigen::Vector3d::Zero()};
  double resolution_;

  // inconfigurable parameters
  // no need to store
  bool received_first_laser_scan_ = false;
  ros::Subscriber laser_sub_;
  tf2_ros::Buffer tf_buffer_ = tf2_ros::Buffer();
  tf2_ros::TransformListener tf_listener_ =
      tf2_ros::TransformListener(tf_buffer_);

  ros::Subscriber wheel_odom_sub_;
  std::pair<double, double> last_wheel_odom_{0.0, 0.0};
  std::pair<double, double> current_wheel_odom_{0.0, 0.0};

  PclCloudPtr last_cloud_{new pcl::PointCloud<pcl::PointXYZ>()};
  std::vector<DreamGMapping::Particle> particles_;
  // transforms to neighbors around a pose estimate
  std::vector<Eigen::Matrix4d> neighbor_transforms_;

  // get the most recent odom -> draw a new noise -> go through all particles,
  void store_last_scan(
      const boost::shared_ptr<const sensor_msgs::LaserScan> &scan_msg);
  void store_last_scan(PclCloudPtr to_update);

  PclCloudPtr
  get_point_cloud_in_world_frame(const SimpleRoboticsCppUtils::Pose2D &pose,
                                 const PclCloudPtr cloud_in_body_frame);

  std::tuple<SimpleRoboticsCppUtils::Pose2D, double, PclCloudPtr>
  optimizeAfterIcp(const DreamGMapping::Particle &particle,
                   const Eigen::Ref<Eigen::Matrix4d> T_icp_output);

  // Note: cloud_in_world_frame is specific to each pose estimate
  void update_particle(const SimpleRoboticsCppUtils::Pose2D &pose,
                       const double &weight,
                       const PclCloudPtr cloud_in_world_frame);
};
} // namespace DreamGMapping