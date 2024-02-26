#pragma once
#include "simple_robotics_cpp_utils/rigid2d.hpp"
#include <boost/smart_ptr/shared_ptr.hpp>
#include <iostream>
#include <pcl/point_cloud.h>
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
// contains pcl::PointXYZ
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>

namespace DreamGMapping {

using PclCloudPtr = boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ>>;
using ScanMsgPtr = boost::shared_ptr<const sensor_msgs::LaserScan>;
using SimpleRoboticsCppUtils::Pixel2DWithCount;
using SimpleRoboticsCppUtils::Pose2D;

class PointAccumulator {
private:
  // { long: {<x,y>, count}} = 8 + 4 + 4 + 4 = 20 bytes. That translates to 20MB
  // of memory. In reality, 67MB for 100 Particles, 10000 points
  std::unordered_map<long, Pixel2DWithCount> count_map_;

public:
  inline void add_point(const unsigned int &x, const unsigned int &y,
                        bool is_hit) {
    const auto key = SimpleRoboticsCppUtils::hash_pixel2d_with_count(x, y);
    if (!count_map_.contains(key)) {

      count_map_.emplace(key, Pixel2DWithCount(x, y));
    }
    if (is_hit) {
      count_map_.at(key).hit_count_++;
    }
    count_map_.at(key).total_count_++;
  }

  inline std::pair<unsigned int, unsigned int>
  get_counts(const unsigned int &x, const unsigned int &y) {
    const auto key = SimpleRoboticsCppUtils::hash_pixel2d_with_count(x, y);
    if (!count_map_.contains(key)) {
      return std::make_pair(0, 0);
    }
    return std::make_pair(count_map_.at(key).hit_count_,
                          count_map_.at(key).total_count_);
  }
};

// motion model update -> pixel value -> transform laser points length /
// resolution * multiplier -> find in point accumulator for scan matching. ->
// T_trans to t_point -> adjust particle pose T_pose -> get T_pose_in_pixel ->
// add tp pointaccumulator laser points transform + T_pose_in_pixel
struct Particle {
  double weight_;
  // refining
  std::vector<std::shared_ptr<SimpleRoboticsCppUtils::Pose2D>> pose_traj_;
  PointAccumulator laser_point_accumulation_map_;
  // all ctor and dtors are default
};

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

}; // namespace DreamGMapping