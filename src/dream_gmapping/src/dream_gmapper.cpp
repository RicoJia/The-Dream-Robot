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
#include <pcl/common/transforms.h>
#include <ros/node_handle.h>
#include <ros/ros.h>
#include <simple_robotics_cpp_utils/rigid2d.hpp>
#include <tuple>

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
  nh_.getParam("resolution", resolution_);

  DreamGMapping::print_all_nodehandle_params(nh_);
  ROS_INFO_STREAM("Successfully read parameters for dream_gmapping");

  // we are not using TimeSynchronizer because tf2 already provides buffering
  // with timestamps
  laser_sub_ = nh_.subscribe("scan", 1, &DreamGMapper::laser_scan, this);
  wheel_odom_sub_ = nh_.subscribe("odom", 1, &DreamGMapper::wheel_odom, this);

  motion_covariances_ << 0.2, 0.0, 0.0, 0.2;
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

  PclCloudPtr cloud(new pcl::PointCloud<pcl::PointXYZ>());
  bool filling_success = DreamGMapping::fill_point_cloud(scan_msg, cloud);
  if (!filling_success) {
    ROS_WARN("Failed to fill point cloud");
    return;
  };

  // - icp: scan match, get initial guess
  Eigen::Matrix4d T_icp_output = Eigen::Matrix4d::Identity();
  bool icp_converge = DreamGMapping::icp_2d(last_cloud_, cloud,
                                            screw_displacement, T_icp_output);

  std::vector<PclCloudPtr> cloud_in_world_frame_vec{};
  cloud_in_world_frame_vec.reserve(particle_num_);

  for (auto &particle : particles_) {
    PclCloudPtr cloud_in_world_frame(new pcl::PointCloud<pcl::PointXYZ>());
    double score;
    SimpleRoboticsCppUtils::Pose2D new_pose_estimate{0, 0, 0};
    // In the if clause, we need: score, the new pose and the new point cloud in
    // world frame
    if (icp_converge) {
      // TODO: not tested yet
      // Unit-testble optimization function
      auto [s, m, cloud_in_world_frame] =
          optimizeAfterIcp(particle, T_icp_output);
    } else {
      // draw from motion model
      auto [s, m] = SimpleRoboticsCppUtils::draw_from_icc(
          *(particle.pose_traj_.back()), screw_displacement,
          motion_covariances_);
      score = s;
      new_pose_estimate = m;
      DreamGMapping::get_point_cloud_in_world_frame(new_pose_estimate,
                                                    cloud_in_world_frame);
    }
    // need to store score, and the new pose now, new point cloud in world frame
    cloud_in_world_frame_vec.push_back(cloud_in_world_frame);
    particle.pose_traj_.back() =
        std::make_shared<SimpleRoboticsCppUtils::Pose2D>(new_pose_estimate);
    particle.weight_ = score;
  }

  // Store world frame point cloud later.
  // - resample based on the scores.
  // update_particle(new_pose_estimate, score, cloud_in_body_frame);
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
  DreamGMapping::fill_point_cloud(scan_msg, last_cloud_);
}
void DreamGMapper::store_last_scan(PclCloudPtr to_update) {
  last_cloud_ = to_update;
}

/**
 * @brief One difference from the original RBPF SLAM paper is that a particle's
 score is not sum_(Probability(pose_sample) * Probability(scan_of_pose_sample))
 over k new samples around particle's pose Instead it's
 max(probability(scan_of_pose_sample)) over k new samples
 *
 * @param particle
 * @param T_icp_output
 * @return std::tuple<SimpleRoboticsCppUtils::Pose2D, double, PclCloudPtr>
 */
std::tuple<SimpleRoboticsCppUtils::Pose2D, double, PclCloudPtr>
DreamGMapper::optimizeAfterIcp(const DreamGMapping::Particle &particle,
                               const Eigen::Ref<Eigen::Matrix4d> T_icp_output) {
  Eigen::Matrix4d new_pose_estimate =
      T_icp_output * (particle.pose_traj_.back()->to_se3());
  double score = 0.0;
  PclCloudPtr cloud;
  // TODO - to move
  std::vector<Eigen::Matrix4d> motion_set_{
      Eigen::Matrix4d::Identity(),
      Eigen::Matrix4d::Identity(), // x forward
      Eigen::Matrix4d::Identity(), // x backward
      Eigen::Matrix4d::Identity(), // y forward
      Eigen::Matrix4d::Identity(), // y backward
      SimpleRoboticsCppUtils::get_transform_from_2d_rotation(
          M_PI_4), // left 45 deg
      SimpleRoboticsCppUtils::get_transform_from_2d_rotation(
          -M_PI_4), // right 45 deg
  };
  motion_set_[1](0, 3) += resolution_;
  motion_set_[2](0, 3) -= resolution_;
  motion_set_[3](1, 3) += resolution_;
  motion_set_[4](1, 3) -= resolution_;
  for (const auto &motion : motion_set_) {
    // search around new_pose_estimate, each neighbor has a point cloud
    auto new_pose_estimate_neighbor = motion * new_pose_estimate;
    PclCloudPtr cloud_in_world_frame_pixelized(
        new pcl::PointCloud<pcl::PointXYZ>());
    DreamGMapping::get_point_cloud_in_world_frame(
        SimpleRoboticsCppUtils::Pose2D(new_pose_estimate_neighbor),
        cloud_in_world_frame_pixelized);
    DreamGMapping::pixelize_point_cloud(cloud_in_world_frame_pixelized,
                                        resolution_);
    for (const auto &endpoint : cloud_in_world_frame_pixelized->points) {
    }
  }
  // TODO
  // cloud_in_world_frame = best_current_neighbor_cloud;
  // DreamGMapping::get_point_cloud_in_world_frame()
  // refine_particle_pose_and_score
  // - for each particle, sample around the scan matched position K times?
  // - Then calculate score for each particle

  return std::make_tuple(SimpleRoboticsCppUtils::Pose2D(new_pose_estimate),
                         score, cloud);
}

// void DreamGMapper::update_particle(
//     const SimpleRoboticsCppUtils::Pose2D &pose, const double &weight,
//     const PclCloudPtr
//         cloud_in_world_frame) {
//             // trans
//         }

} // namespace DreamGMapping