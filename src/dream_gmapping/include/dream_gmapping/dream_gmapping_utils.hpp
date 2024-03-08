#pragma once
#include "simple_robotics_cpp_utils/rigid2d.hpp"
#include <boost/smart_ptr/shared_ptr.hpp>
#include <iostream>
#include <pcl/point_cloud.h>
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
// contains pcl::PointXYZ
#include <nav_msgs/OccupancyGrid.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <unordered_map>
#include <vector>

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

  /**
   * @brief we are returning if a pixel is full by a set standard. (if the pixel
   * is unknown, it is not full)
   * TODO: check if we need to change the unknown stage
   *
   * @param p
   */
  inline bool is_full(const SimpleRoboticsCppUtils::Pixel2DWithCount &p) const {
    auto [hit_count, total_count] = get_counts(p.x, p.y);
    // We set 0.5 as known/unknown boundary.
    return (hit_count << 1) > total_count;
  }

  /**
   * @brief Return a pixel's counts of hits and total counts. If the pixel
   * doesn't exist, return 0, 0
   *
   * @param x
   * @param y
   * @return std::pair<unsigned int, unsigned int> : hit_count, total_count
   */
  inline std::pair<unsigned int, unsigned int>
  get_counts(const unsigned int &x, const unsigned int &y) const {
    const auto key = SimpleRoboticsCppUtils::hash_pixel2d_with_count(x, y);
    if (!count_map_.contains(key)) {
      return std::make_pair(0, 0);
    }
    return std::make_pair(count_map_.at(key).hit_count_,
                          count_map_.at(key).total_count_);
  }

  // Origin is at the center of the map, with the bottom left corner of the map
  // being at (0,0) Origin offset is the index of the origin in the 1D array
  inline void fill_ros_map(std::vector<int8_t> &data,
                           const unsigned int &map_size,
                           const unsigned int &origin_offset) const {
    data = std::vector<int8_t>(map_size * map_size, -1);
    for (const auto &pair : count_map_) {
      const auto &pixel = pair.second;
      const auto &hit_count = pixel.hit_count_;
      const auto &total_count = pixel.total_count_;
      // add each point to the map TODO. Obstacle is 100, free is 0
      const int index = pixel.y * static_cast<int>(map_size) + pixel.x +
                        static_cast<int>(origin_offset);
      if ((hit_count << 1) > total_count) {
        data.at(index) = 100;
      } else {
        data.at(index) = 0;
      }
    }
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
                   const Eigen::Matrix4d &T_init_guess_double,
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
  // Documentation on T_init_guess being float typed sucked - I haven't seen it
  auto T_init_guess = T_init_guess_double.cast<float>();
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

inline void find_most_weighted_particle_index(
    const std::unordered_map<unsigned int, unsigned int> counts,
    unsigned int &best_particle_index) {
  unsigned int best_particle_count = 0;
  for (const auto &count_pair : counts) {
    if (count_pair.second > best_particle_count) {
      best_particle_count = count_pair.second;
      best_particle_index = count_pair.first;
    }
  }
}

inline void
find_most_weighted_particle_index(const std::vector<Particle> &particles,
                                  unsigned int &best_particle_index) {
  double best_particle_weight = 0;
  for (unsigned int i = 0; i < particles.size(); i++) {
    if (particles[i].weight_ > best_particle_weight) {
      best_particle_weight = particles[i].weight_;
      best_particle_index = i;
    }
  }
}

}; // namespace DreamGMapping