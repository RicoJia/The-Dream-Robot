/** @file
    @brief Rao-Blackwellized Gmapping, with the counting mapping model
    @author Rico Ruotong Jia
    @date 2024-01-28
    SUBSCRIBES:
    - @subscribes: /scan: laser scan data
    - @subscribes: /odom: wheel positions in m/s. [left, right]
    - @subscribes: /tf: listens for for odom -> baselink
    PUBLISHES:
    - @publishes: /map: map of the current environment
    - @publishes: /tf: map -> odom transform

    PARAMETERS:
    - @param: base_frame
    - @param: map_frame
    - @param: odom_frame
    - @param: map_update_interval
    laser params
    - @param: maxRange: maximum laser scanner range
    - @param: sigma: standard deviation for the scan matching process
    - @param: search window for scan matching
    // TODO: scan matching kernel, and steps?
    // TODO: benefits of
        scan_filter_sub_ = new
   message_filters::Subscriber<sensor_msgs::LaserScan>(node_, "scan", 5);
        scan_filter_ = new
   tf::MessageFilter<sensor_msgs::LaserScan>(*scan_filter_sub_, tf_,
   odom_frame_, 5);
        scan_filter_->registerCallback(boost::bind(&SlamGMapping::laserCallback,
   this, _1));
*/

// Need message filter. Also, a 1s "expiration time"?
#include "dream_gmapping/dream_gmapper.hpp"
#include "simple_robotics_cpp_utils/math_utils.hpp"
#include "tf2/exceptions.h"
#include <iostream>
#include <memory>
#include <ros/node_handle.h>
#include <ros/ros.h>

namespace DreamGMapping {
DreamGMapper::DreamGMapper(ros::NodeHandle nh_) {
  // Read parameters without default values here, so they must be specified in
  // launch file
  nh_.getParam("base_frame", base_frame_);
  nh_.getParam("map_frame", map_frame_);
  nh_.getParam("odom_frame", odom_frame_);
  nh_.getParam("map_update_interval", map_update_interval_);

  nh_.getParam("max_range", max_range_);
  nh_.getParam("particle_num", particle_num_);

  RosUtils::print_all_nodehandle_params(nh_);
  ROS_INFO_STREAM("Successfully read parameters for dream_gmapping");

  // we are not using TimeSynchronizer because tf2 already provides buffering
  // with timestamps
  laser_sub_ = nh_.subscribe("scan", 1, &DreamGMapper::laser_scan, this);
  wheel_odom_sub_ = nh_.subscribe("odom", 1, &DreamGMapper::wheel_odom, this);

  motion_covariances_ << 0.2, 0.0, 0.0, 0.0, 0.2, 0.0, 0.0, 0.0, 0.1;
  motion_means_ << 0.0, 0.0, 0.0;

  // std::shared_ptr<Rigid2D::>
  for (unsigned int i = 0; i < particle_num_; i++) {
    // TODO: check if weight is 1/particle_num;
    particles_.push_back(Particle{1.0 / particle_num_});
  }

  ROS_INFO_STREAM("Successfully initialized dream_gmapping");
}

DreamGMapper::~DreamGMapper() = default;

void DreamGMapper::laser_scan(
    const boost::shared_ptr<const sensor_msgs::LaserScan> &scan_msg) {
  // - wait for the first scan message, get tf;

  geometry_msgs::TransformStamped odom_to_base;
  try {
    odom_to_base =
        tf_buffer_.lookupTransform(base_frame_, odom_frame_, ros::Time(0));
    auto received_time = (ros::Time(0) - odom_to_base.header.stamp).toSec();
    if (received_time > 1.0) {
      ROS_WARN_STREAM("Odom to base transform was received " << received_time);
    }
  } catch (tf2::TransformException &e) {
    ROS_WARN("%s", e.what());
    return;
  }

  // TODO: test the filter, with a laserscan msg first, then with odom
  if (!received_first_laser_scan_) {
    // Store the laser->base transform
    received_first_laser_scan_ = true;
    store_last_scan(scan_msg);
    ROS_DEBUG_STREAM("Received first laser scan");
    return;
  }

  // if scan is shorter than range_min, then set it to range_max
  // Then make a copy of the scan. TODO: can we keep the scan msg?
  // add_scan();

  // add_scan() needs to push the 360 values into particles. Then when update
  // maps, we go over the dictionary.

  // process_scan()
  // - icp: scan match, get initial guess
  // for (particle& : particle_set){
  //     update motion model(), get a new draw
  // }
  // - scan_match() // each particle's map with the new draw, get a score
  // - for each particle, sample around the scan matched position K times?
  // - Then calculate score for each particle
  // - resample based on the scores.
  // - map update
  store_last_scan(scan_msg);
}
void wheel_odom(const std_msgs::Float32MultiArray::ConstPtr &odom_msg){
    // TODO
}
void DreamGMapper::store_last_scan(
    const boost::shared_ptr<const sensor_msgs::LaserScan> &scan_msg) {
  last_scan_.clear();
  last_scan_.reserve(scan_msg->ranges.size());
  // create a copy because scan_msg is managed by ROS and could be destroyed
  // afterwards
  last_scan_.assign(scan_msg->ranges.begin(), scan_msg->ranges.end());
}
void DreamGMapper::update_with_motion_model() {
  for (Particle &p : particles_) {
    //   p.motion_model();
  }
}
} // namespace DreamGMapping
