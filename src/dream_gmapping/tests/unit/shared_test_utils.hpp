#pragma once

#include "dream_gmapping/dream_gmapping_utils.hpp"
#include "simple_robotics_cpp_utils/performance_utils.hpp"

constexpr double WHEEL_DIST = 1;
constexpr double WHEEL_DIAMETER = 0.3;
constexpr int LARGE_PARTICLE_NUM = 1000;
constexpr int SMALL_PARTICLE_NUM = 10;
constexpr double D_V_STD_DEV = 0.1;
constexpr double D_THETA_STD_DEV = 0.1;
constexpr double RESOLUTION = 0.05;
constexpr double BEAM_NOISE_VARIANCE = 0.02;
constexpr double BEAM_KERNEL_SIZE = 0.05;
// unsigned int is not natively supported by ROS
constexpr int MAP_SIZE_IN_METERS = 10.0;
constexpr double TRANSLATION_ACTIVE_THRESHOLD = 0.1;
constexpr double ANGULAR_ACTIVE_THRESHOLD = 0.1;
constexpr double INITIAL_WALL_DIST = 1.0;
constexpr double WALL_LENGTH = 1.0;
constexpr double MAX_SCAN_RANGE = 3.0;
const char *WHEEL_POS_TOPIC("wheel_positions");
const char *SCAN_TOPIC("scan");
const char *MAP_TOPIC("/map");

// Creating a "wall" that's WALL_LENGTH long along the y axis at x=distance
// The laser scan->frame on laser scan->with a. specified distance
// Note: the other half of the scan is max_distance
inline sensor_msgs::LaserScan::ConstPtr
create_wall_laser_scan(const double &distance) {
  boost::shared_ptr<sensor_msgs::LaserScan> scan(new sensor_msgs::LaserScan());
  constexpr int NUM_POINTS = 360;
  scan->ranges = std::vector<float>();
  scan->header.frame_id = "laser";
  scan->angle_min = 0;
  scan->angle_max = 2 * M_PI;
  // Positive is counter clockwise
  scan->angle_increment = 2 * M_PI / NUM_POINTS;
  scan->range_min = 0.0;
  scan->range_max = MAX_SCAN_RANGE;
  double angle = scan->range_min;
  for (unsigned int i = 0; i < NUM_POINTS; i++) {
    if (std::abs(std::cos(angle)) < std::numeric_limits<float>::epsilon()) {
      scan->ranges.push_back(scan->range_max);
    } else if (M_PI / 2.0 <= angle && angle <= M_PI * 3.0 / 2.0) {
      scan->ranges.push_back(scan->range_max);
    } else {
      // the wall is WALL/LENGTH/2.0 on each side
      if (distance * std::abs(std::sin(angle) / std::cos(angle)) <
          WALL_LENGTH / 2.0) {
        scan->ranges.push_back(distance / std::cos(angle));
      } else {
        scan->ranges.push_back(scan->range_max);
      }
    }
    angle += scan->angle_increment;
  }
  return sensor_msgs::LaserScan::ConstPtr(scan);
}