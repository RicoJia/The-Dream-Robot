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
#include <simple_robotics_cpp_utils/rigid2d.hpp>

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
  nh_.getParam("wheel_dist", wheel_dist_);
  nh_.getParam("angular_active_threshold", angular_active_threshold_);
  nh_.getParam("translation_active_threshold", translation_active_threshold_);

  double d_v_std_dev, d_theta_std_dev;
  nh_.getParam("d_v_std_dev", d_v_std_dev);
  nh_.getParam("d_theta_std_dev", d_theta_std_dev);
  motion_covariances_ << d_v_std_dev, 0.0, 0.0, d_theta_std_dev;

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

  if (!received_first_laser_scan_) {
    // Store the laser->base transform
    received_first_laser_scan_ = true;
    store_last_scan(scan_msg);
    ROS_DEBUG_STREAM("Received first laser scan");
    return;
  }

  auto screw_displacement = SimpleRoboticsCppUtils::get_2D_screw_displacement(
      current_wheel_odom_, wheel_dist_);
  auto [d_v, d_theta] = screw_displacement;

  // If odom, angular distance is not enough, skip.
  if (d_v < translation_active_threshold_ &&
      d_theta < angular_active_threshold_)
    return;

  boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> cloud(
      new pcl::PointCloud<pcl::PointXYZ>());
  bool filling_success = RosUtils::fill_point_cloud(scan_msg, cloud);
  if (!filling_success) {
    ROS_WARN("Failed to fill point cloud");
    return;
  };

  // - icp: scan match, get initial guess
  Eigen::Matrix4d T_icp_output = Eigen::Matrix4d::Identity();
  bool icp_converge =
      RosUtils::icp_2d(last_cloud_, cloud, screw_displacement, T_icp_output);

  for (auto &particle : particles_) {
    double score;
    SimpleRoboticsCppUtils::Pose2D new_pose_estimate{0, 0, 0};
    if (icp_converge) {
      // TODO
      // new_pose *= icp
      // search in neighbor
    // refine_particle_pose_and_score
    } else {
      // draw from motion model
      auto [s, m] =
          SimpleRoboticsCppUtils::draw_from_icc(
              *(particle.pose_traj_.back()), screw_displacement, motion_covariances_);
      score = s;
      new_pose_estimate = m;
    }
  }

  // for (particle& : particle_set){
  //     update motion model(), get a new draw
  // }
  // - scan_match() // each particle's map with the new draw, get a score
  // - for each particle, sample around the scan matched position K times?
  // - Then calculate score for each particle
  // - resample based on the scores.
  // - map update
  //   store_last_scan(cloud);
}

// [left, right], the wheel positions are [0, 2pi]
void DreamGMapper::wheel_odom(
    const std_msgs::Float32MultiArray::ConstPtr &odom_msg) {
  // No mutex is needed as we are using ros::spin()
  current_wheel_odom_ = {odom_msg->data[0], odom_msg->data[1]};
}

void DreamGMapper::store_last_scan(
    const boost::shared_ptr<const sensor_msgs::LaserScan> &scan_msg) {
  RosUtils::fill_point_cloud(scan_msg, last_cloud_);
}
void DreamGMapper::store_last_scan(
    boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> to_update) {
  last_cloud_ = to_update;
}

void DreamGMapper::update_with_motion_model() {
  for (Particle &p : particles_) {
    //   p.motion_model();
  }
}
} // namespace DreamGMapping
