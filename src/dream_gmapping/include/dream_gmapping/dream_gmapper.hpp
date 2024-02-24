#pragma once
#include "dream_gmapping/particle_filter.hpp"
#include <boost/smart_ptr/shared_ptr.hpp>
#include <cmath>
#include <geometry_msgs/TransformStamped.h>
#include <memory>
#include <message_filters/subscriber.h>
#include <ros/node_handle.h>
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <simple_robotics_cpp_utils/math_utils.hpp>
#include <simple_robotics_cpp_utils/rigid2d.hpp>
#include <std_msgs/Float32MultiArray.h>
#include <tf2_ros/message_filter.h>
#include <tf2_ros/transform_listener.h>
#include <vector>

#include <pcl/point_cloud.h>
// contains pcl::PointXYZ
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>

using PclCloudPtr = boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ>>;

namespace RosUtils {
inline void print_all_nodehandle_params(ros::NodeHandle nh) {
  std::vector<std::string> params;
  nh.getParamNames(params);
  for (auto &param : params) {
    std::string param_val;
    nh.getParam(param, param_val);
    ROS_INFO_STREAM("param: " << param << " val: " << param_val);
  }
}
// Returning if filling is successful
inline bool fill_point_cloud(
    const boost::shared_ptr<const sensor_msgs::LaserScan> &scan_msg,
    PclCloudPtr cloud) {
  if (!cloud) {
    // create PclCloudPtr
    PclCloudPtr cloud(new pcl::PointCloud<pcl::PointXYZ>);
  }
  // no NaN and INF
  cloud->is_dense = true;
  cloud->height = 1;
  cloud->width = scan_msg->ranges.size();
  cloud->resize(cloud->width);

  if (cloud->width == 0)
    return false;

  const double ANGLE_INCREMENT =
      (std::abs(scan_msg->angle_increment - 0) < 1e-7)
          ? SimpleRoboticsCppUtils::TWO_M_PI / cloud->width
          : scan_msg->angle_increment;
  double angle = 0.0;
  for (unsigned int i = 0; i < cloud->width; i++) {
    // if scan is shorter than range_min, then set it to range_max
    double range = (scan_msg->ranges[i] < scan_msg->range_min)
                       ? scan_msg->range_max
                       : scan_msg->ranges[i];
    cloud->points[i].x = range * std::cos(angle);
    cloud->points[i].y = range * std::sin(angle);
    angle += ANGLE_INCREMENT;
  }
  return true;
}

// screw_displacement is [d_v, d_w]
inline bool icp_2d(const PclCloudPtr prev_scan, const PclCloudPtr curr_scan,
                   const std::pair<double, double> &screw_displacement,
                   Eigen::Matrix4d &T_icp_output) {
  pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
  // pcl will try to align source to target. That's counter to our motion
  icp.setInputCloud(curr_scan);
  icp.setInputTarget(prev_scan);
  icp.setMaximumIterations(1500);     // A higher number of iterations
  icp.setTransformationEpsilon(1e-6); // A smaller convergence threshold
  icp.setRANSACOutlierRejectionThreshold(0.05);
  // potentially decrease in a loop
  icp.setEuclideanFitnessEpsilon(
      1e-5); // A smaller distance threshold for stopping
  pcl::PointCloud<pcl::PointXYZ> output;
  // TODO: get Transform from screw_displacement
  auto T_init_guess =
      SimpleRoboticsCppUtils::screw_displacement_2d_to_body_frame_transform(
          screw_displacement)
          .cast<float>();
  // Documentation on T_init_guess being float typed sucked - I haven't seen it
  icp.align(output, T_init_guess);

  if (icp.hasConverged()) {
    T_icp_output = icp.getFinalTransformation().cast<double>();
    return true;
  } else {
    ROS_WARN("ICP did not converge");
    return false;
  }
}

inline PclCloudPtr
get_point_cloud_in_world_frame(const SimpleRoboticsCppUtils::Pose2D &pose,
                               const PclCloudPtr cloud_in_body_frame) {
  auto pose_eigen4d = pose.to_se3();
  PclCloudPtr cloud_in_world_frame{new pcl::PointCloud<pcl::PointXYZ>()};
  pcl::transformPointCloud(*cloud_in_body_frame, *cloud_in_world_frame,
                           pose_eigen4d);
  // TODO, pixelize the cloud????
  // TODO: write a test?
  return cloud_in_world_frame;
}

/**
 * @brief Pixelize a point cloud, regardless of the frame it's in
 *
 * @param cloud - pcl Point cloud
 * @param resolution - resolution in meters
 */
inline void pixelize_point_cloud(PclCloudPtr cloud, const double &resolution) {
  for (auto &point : cloud->points) {
    point.x = static_cast<int>(std::floor(point.x / resolution));
    point.y = static_cast<int>(std::floor(point.y / resolution));
  }
}

} // namespace RosUtils

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