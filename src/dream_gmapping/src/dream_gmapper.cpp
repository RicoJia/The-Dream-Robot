/** @file
    @brief Rao-Blackwellized Gmapping, with the counting mapping model
    @author Rico Ruotong Jia
    @date 2024-01-28
    SUBSCRIBES:
    - @subscribes: /scan: laser scan data
    - @subscribes: /odom: wheel positions in m/s. [left, right]
    - @subscribes: /tf: listens for for odom -> baselink
    PUBLISHES:
    - @publishes: /map: map of the current environment. Note, currently map is
   fixed size and does NOT size up
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
*/

// Need message filter. Also, a 1s "expiration time"?
#include "dream_gmapping/dream_gmapper.hpp"
#include "dream_gmapping/dream_gmapping_utils.hpp"
#include "geometry_msgs/TransformStamped.h"
#include "nav_msgs/OccupancyGrid.h"
#include "ros/duration.h"
#include "simple_robotics_cpp_utils/math_utils.hpp"
#include "tf2/exceptions.h"
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <eigen3/Eigen/src/Core/Matrix.h>
#include <iostream>
#include <limits>
#include <memory>
#include <numeric>
#include <pcl/common/transforms.h>
#include <pcl_conversions/pcl_conversions.h>
#include <random>
#include <ros/node_handle.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <simple_robotics_cpp_utils/rigid2d.hpp>
#include <tf2_eigen/tf2_eigen.h>
#include <tuple>
#include <unordered_map>
#include <utility>
#include <vector>

namespace DreamGMapping {
DreamGMapper::DreamGMapper(ros::NodeHandle nh_) {
  // Read parameters without default values here, so they must be specified in
  // launch file
  nh_.getParam("base_frame", base_frame_);
  nh_.getParam("map_frame", map_frame_);
  nh_.getParam("odom_frame", odom_frame_);
  nh_.getParam("scan_frame", scan_frame_);
  nh_.getParam("map_update_interval", map_update_interval_);

  nh_.getParam("max_range", max_range_);
  nh_.getParam("particle_num", particle_num_);
  nh_.getParam("angular_active_threshold", angular_active_threshold_);
  nh_.getParam("translation_active_threshold", translation_active_threshold_);

  double d_v_std_dev, d_theta_std_dev;
  nh_.getParam("d_v_std_dev", d_v_std_dev);
  nh_.getParam("d_theta_std_dev", d_theta_std_dev);
  motion_covariances_ << d_v_std_dev * d_v_std_dev, 0.0, 0.0,
      d_theta_std_dev * d_theta_std_dev;
  motion_means_ << 0.0, 0.0, 0.0;
  nh_.getParam("resolution", resolution_);
  nh_.getParam("beam_noise_variance", beam_noise_variance_);
  nh_.getParam("half_beam_kernel_size", half_beam_kernel_size_);
  nh_.getParam("skip_invalid_beams", skip_invalid_beams_);
  std::string scan_topic, map_topic;
  nh_.getParam("scan_topic", scan_topic);
  nh_.getParam("map_topic", map_topic);
  double map_size_in_meters;
  nh_.getParam("map_size_in_meters", map_size_in_meters);
  nh_.getParam("publish_debug_scan", publish_debug_scan_);
  int pose_correction_kernel_size;
  nh_.param("pose_correction_kernel_size", pose_correction_kernel_size, 2);
  nh_.getParam("occupied_fullness_threshold", occupied_fullness_threshold_);

  map_size_ = static_cast<unsigned int>(map_size_in_meters / resolution_);
  if (map_size_ % 2 == 0) {
    map_size_++;
    ROS_WARN_STREAM(
        "To have an odd number of cells in the map, map size now is "
        << map_size_ * resolution_);
  }
  map_pub_ = nh_.advertise<nav_msgs::OccupancyGrid>(map_topic, 1);
  initialize_map();

  //   DreamGMapping::print_all_nodehandle_params(nh_);
  ROS_INFO_STREAM("Successfully read parameters for dream_gmapping");

  // we are not using TimeSynchronizer because tf2 already provides buffering
  // with timestamps
  laser_sub_ = nh_.subscribe(scan_topic, 1, &DreamGMapper::laser_scan, this);

  for (unsigned int i = 0; i < particle_num_; i++) {
    // TODO: check if weight is 1/particle_num;
    particles_.push_back(Particle{
        1.0 / particle_num_,
        {std::make_shared<SimpleRoboticsCppUtils::Pose2D>(0.0, 0.0, 0.0)}});
  }

  initialize_motion_set(pose_correction_kernel_size);
  const int kernel_size = 2 * half_beam_kernel_size_ + 1;
  beam_search_kernel_ = std::vector<int>(kernel_size, 0);
  for (int i = 1; i < half_beam_kernel_size_ + 1; ++i) {
    beam_search_kernel_[2 * i - 1] = i;
    beam_search_kernel_[2 * i] = -i;
  }
  const double external_kernel_diagnol_squared =
      std::pow(resolution_ * (10 + half_beam_kernel_size_), 2);
  prob_not_found_ =
      std::exp(-external_kernel_diagnol_squared / beam_noise_variance_);

  // initialize debugging publishers
  if (publish_debug_scan_) {
    last_cloud_debug_scan_pub_ =
        nh_.advertise<sensor_msgs::PointCloud2>("last_cloud_debug_scan", 1);
    current_cloud_debug_scan_pub_ =
        nh_.advertise<sensor_msgs::PointCloud2>("current_cloud_debug_scan", 1);
  }
  ROS_INFO_STREAM("Successfully initialized dream_gmapping");
}

DreamGMapper::~DreamGMapper() = default;

void DreamGMapper::initialize_map() {
  // we know map_size_ is an odd number
  map_.header.frame_id = "map";
  map_.info.resolution = resolution_;
  map_.info.width = map_size_;
  map_.info.height = map_size_;
  // this is the real world pose of (0, 0) of the map.
  map_.info.origin.position.x =
      -(static_cast<int>(map_size_)) / 2 * resolution_;
  map_.info.origin.position.y =
      -(static_cast<int>(map_size_)) / 2 * resolution_;
  map_.info.origin.position.z = 0.0;
  origin_offset_ = map_size_ * map_size_ / 2;
  map_.info.origin.orientation.w = 1.0;

  map_to_odom_tf_.header.frame_id = "map";
  map_to_odom_tf_.child_frame_id = "odom";
  map_to_odom_tf_.transform.rotation.w = 1.0;
}

void DreamGMapper::initialize_motion_set(
    const int &optimization_steps_iteration) {
  auto get_matrix = [&](const double &x, const double &y, const double &angle) {
    Eigen::Matrix4d T =
        SimpleRoboticsCppUtils::get_transform_from_2d_rotation(angle);
    T(0, 3) += x;
    T(1, 3) += y;
    return T;
  };

  double angle_step = resolution_;
  double linear_step = resolution_;
  // this matrix has the pose itself first
  optimization_transforms_vec_.push_back({get_matrix(0, 0, 0)});
  for (int i = 0; i < optimization_steps_iteration; ++i) {
    // these are base frame transforms, for moving right, left, forward,
    // backward, right, left
    optimization_transforms_vec_.push_back({
        get_matrix(0, 0, angle_step), get_matrix(0, 0, -angle_step),
        get_matrix(linear_step, 0, 0), get_matrix(-linear_step, 0, 0),
        get_matrix(0, linear_step, 0), get_matrix(0, -linear_step, 0),

        get_matrix(linear_step, 0, angle_step),
        get_matrix(linear_step, 0, -angle_step),
        get_matrix(-linear_step, 0, angle_step),
        get_matrix(-linear_step, 0, -angle_step),
        get_matrix(0, linear_step, angle_step),
        get_matrix(0, linear_step, -angle_step),
        get_matrix(0, -linear_step, angle_step),
        get_matrix(0, -linear_step, -angle_step),
    });
    angle_step *= 0.5;
    linear_step *= 0.5;
  }
}

bool DreamGMapper::initialize_base_to_scan() {
  if (base_to_scan_.isApprox(Eigen::Matrix4d::Zero())) {
    try {
      auto base_to_scan_tf =
          tf_buffer_.lookupTransform(base_frame_, scan_frame_, ros::Time(0));
      Eigen::Affine3d transform_eigen =
          tf2::transformToEigen(base_to_scan_tf.transform);
      base_to_scan_ = transform_eigen.matrix();
      base_to_scan_(2, 3) = 0.0; // setting z to 0
      std::cout << "base_to_scan_:" << std::endl << base_to_scan_ << std::endl;
    } catch (tf2::TransformException &e) {
      ROS_WARN("%s", e.what());
      return false;
    }
  }
  return true;
}
bool DreamGMapper::get_odom_to_base(Eigen::Matrix4d &T_delta,
                                    Eigen::Matrix4d &current_odom_pose) {
  // - wait for the first scan message, get tf;
  geometry_msgs::TransformStamped odom_to_base;
  try {
    odom_to_base =
        tf_buffer_.lookupTransform(odom_frame_, base_frame_, ros::Time(0));
    auto received_time = (ros::Time(0) - odom_to_base.header.stamp).toSec();
    if (received_time > 1.0) {
      ROS_WARN_STREAM("Odom to base transform was received " << received_time);
      return false;
    }
  } catch (tf2::TransformException &e) {
    ROS_WARN("%s", e.what());
    return false;
  }
  if (!initialize_base_to_scan())
    return false;

  /**odom_to_base to eigen -> calculate T_delta -> store in last_odom_pose*/
  Eigen::Affine3d transform_eigen =
      tf2::transformToEigen(odom_to_base.transform);
  current_odom_pose = transform_eigen.matrix();
  T_delta = last_odom_pose_.inverse() * current_odom_pose;
  return true;
}

void DreamGMapper::laser_scan(
    const boost::shared_ptr<const sensor_msgs::LaserScan> &scan_msg) {
  std::cout << "==========================================" << std::endl;
  ros::Time current_time = ros::Time::now();

  // T_delta -> delta_translation, delta_rotation
  Eigen::Matrix4d T_delta, current_odom_pose;
  if (!get_odom_to_base(T_delta, current_odom_pose))
    return;
  double delta_translation =
      SimpleRoboticsCppUtils::get_norm_of_translation(T_delta);
  double delta_theta =
      SimpleRoboticsCppUtils::get_2d_rotation_from_z_axis(T_delta);

  /**Prepare cloud in robot body frame, and initialize new pose estimate*/
  // this point cloud will be needed by everybody, fella.
  PclCloudPtr cloud_in_body_frame(new pcl::PointCloud<pcl::PointXYZ>());
  bool filling_success = DreamGMapping::fill_point_cloud(
      scan_msg, cloud_in_body_frame, skip_invalid_beams_);
  if (!filling_success) {
    ROS_WARN("Failed to fill point cloud");
    return;
  };
  cloud_in_body_frame = DreamGMapping::transform_point_cloud_eigen4d(
      base_to_scan_, cloud_in_body_frame);
  PclCloudPtr cloud_in_world_frame(new pcl::PointCloud<pcl::PointXYZ>());
  SimpleRoboticsCppUtils::Pose2D new_pose_estimate{0, 0, 0};

  if (!received_first_laser_scan_) {
    last_odom_pose_ = current_odom_pose;
    for (auto &particle : particles_) {
      new_pose_estimate = SimpleRoboticsCppUtils::Pose2D(
          particle.pose_traj_.back()->to_se3() * T_delta);
      cloud_in_world_frame = DreamGMapping::transform_point_cloud(
          new_pose_estimate, cloud_in_body_frame);
      DreamGMapping::pixelize_point_cloud(cloud_in_world_frame, resolution_);
      particle.pose_traj_.back() =
          std::make_shared<SimpleRoboticsCppUtils::Pose2D>(new_pose_estimate);
    }
    received_first_laser_scan_ = true;
    ROS_INFO_STREAM("Received first laser scan");
    std::cout << "Received first laser scan" << std::endl;
    resample_if_needed_and_update_particle_map_and_find_best_pose(scan_msg);
    publish_map();
    store_last_scan(cloud_in_body_frame);
  } else {
    // If odom, angular distance is not enough, skip.
    if (std::abs(delta_translation) < translation_active_threshold_ &&
        std::abs(delta_theta) < angular_active_threshold_)
      return;
    last_odom_pose_ = current_odom_pose;
    // TODO
    std::cout << "current_odom_pose: " << current_odom_pose << std::endl;

    for (auto &particle : particles_) {
      // In the if clause, we need: score, the new pose and the new point cloud
      // in world frame

      // TODO: Better noise model
      *particle.pose_traj_.back() = SimpleRoboticsCppUtils::Pose2D(
          particle.pose_traj_.back()->to_se3() * T_delta);

      //   SimpleRoboticsCppUtils::draw_from_icc(
      //       *particle.pose_traj_.back(), {delta_translation, delta_theta},
      //       motion_covariances_);
      //   *particle.pose_traj_.back() = new_motion_pose;
    }

    if ((ros::Time::now() - last_map_update_).toSec() > map_update_interval_) {
      for (auto &particle : particles_) {
        // Unit-testble optimization function
        auto [m, s, c] =
            gradient_descent_optimize(particle, T_delta, cloud_in_body_frame);
        double score = s;
        new_pose_estimate = m;
        cloud_in_world_frame = c;

        particle.pose_traj_.back() =
            std::make_shared<SimpleRoboticsCppUtils::Pose2D>(new_pose_estimate);
        particle.weight_ = score;
      }
      resample_if_needed_and_update_particle_map_and_find_best_pose(scan_msg);
      publish_map();
      store_last_scan(cloud_in_body_frame);
    }
  }

  ros::Duration delta = ros::Time::now() - current_time;
  // TODO
  std::cout << "time delta: " << delta.toSec() << std::endl;
}

void DreamGMapper::store_last_scan(
    const boost::shared_ptr<const sensor_msgs::LaserScan> &scan_msg,
    const bool &skip_invalid_beams) {
  DreamGMapping::fill_point_cloud(scan_msg, last_cloud_, skip_invalid_beams);
}
void DreamGMapper::store_last_scan(PclCloudPtr to_update) {
  last_cloud_ = to_update;
}

void DreamGMapper::publish_debug_scans(PclCloudPtr last_cloud,
                                       PclCloudPtr current_cloud) {
  auto get_scan_msg = [&](PclCloudPtr cloud) {
    sensor_msgs::PointCloud2 scan_msg;
    pcl::toROSMsg(*cloud, scan_msg);
    scan_msg.header.frame_id = map_frame_;
    return scan_msg;
  };
  if (publish_debug_scan_) {
    last_cloud_debug_scan_pub_.publish(get_scan_msg(last_cloud_));
    // TODO
    // current_cloud_debug_scan_pub_.publish(get_scan_msg(current_cloud));
  }
}

std::tuple<SimpleRoboticsCppUtils::Pose2D, double, PclCloudPtr>
DreamGMapper::gradient_descent_optimize(
    const DreamGMapping::Particle &particle,
    const Eigen::Ref<Eigen::Matrix4d> T_icp_output,
    PclCloudPtr cloud_in_body_frame) {

  int i = 0;
  double best_score = -1.0;
  PclCloudPtr best_cloud(new pcl::PointCloud<pcl::PointXYZ>());
  auto best_pose = *particle.pose_traj_.back();
  std::cout << "before optimization: " << std::endl << best_pose;
  while (i < optimization_transforms_vec_.size()) {
    double best_local_score = -1.0;
    PclCloudPtr best_local_cloud(new pcl::PointCloud<pcl::PointXYZ>());
    auto best_local_pose = best_pose;
    for (const auto &trans : optimization_transforms_vec_.at(i)) {
      auto new_pose_estimate_neighbor =
          SimpleRoboticsCppUtils::Pose2D(trans * best_local_pose.to_se3());
      PclCloudPtr cloud_in_world_frame_pixelized(
          new pcl::PointCloud<pcl::PointXYZ>());
      // we are comparing the valid point cloud
      cloud_in_world_frame_pixelized = DreamGMapping::transform_point_cloud(
          new_pose_estimate_neighbor, cloud_in_body_frame);
      DreamGMapping::pixelize_point_cloud(cloud_in_world_frame_pixelized,
                                          resolution_);
      double score = observation_model_score(
          cloud_in_world_frame_pixelized, new_pose_estimate_neighbor,
          particle.laser_point_accumulation_map_);
      if (score > best_local_score) {
        best_local_score = score;
        best_local_cloud = cloud_in_world_frame_pixelized;
        best_local_pose = new_pose_estimate_neighbor;
      }
      std::cout << "score: " << score
                << ", new_pose estimate: " << new_pose_estimate_neighbor
                << std::endl;
    }
    if (best_score >= best_local_score) {
      // when our current transform is not yielding a better result,
      ++i;
    } else {
      best_score = best_local_score;
      best_cloud = best_local_cloud;
      best_pose = best_local_pose;
      // move on since we are at i=0 since it's identity transform
      if (i == 0)
        ++i;
    }
    // TODO
    std::cout << "i: " << i << "score" << best_score << ", best pose"
              << std::endl
              << best_pose << std::endl;
    // TODO
    std::cout << "==========" << std::endl;
  }
  return std::make_tuple(best_pose, best_score, best_cloud);
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
DreamGMapper::optimize_after_icp(const DreamGMapping::Particle &particle,
                                 const Eigen::Ref<Eigen::Matrix4d> T_icp_output,
                                 PclCloudPtr cloud_in_body_frame) {
  Eigen::Matrix4d new_pose_estimate =
      T_icp_output * (particle.pose_traj_.back()->to_se3());
  double score = 0.0;
  PclCloudPtr best_cloud(new pcl::PointCloud<pcl::PointXYZ>());
  double best_score = std::numeric_limits<double>::lowest();
  Pose2D best_pose(0, 0, 0);
  for (const auto &motion : motion_set_) {

    // search around new_pose_estimate, each neighbor has a pixelized pointcloud
    auto new_pose_estimate_neighbor =
        SimpleRoboticsCppUtils::Pose2D(motion * new_pose_estimate);

    PclCloudPtr cloud_in_world_frame_pixelized(
        new pcl::PointCloud<pcl::PointXYZ>());
    // we are comparing the valid point cloud
    cloud_in_world_frame_pixelized = DreamGMapping::transform_point_cloud(
        new_pose_estimate_neighbor, cloud_in_body_frame);
    DreamGMapping::pixelize_point_cloud(cloud_in_world_frame_pixelized,
                                        resolution_);

    double score = observation_model_score(
        cloud_in_world_frame_pixelized, new_pose_estimate_neighbor,
        particle.laser_point_accumulation_map_);
    std::cout << "score returned from observation model score: " << score
              << std::endl;
    if (score > best_score) {
      best_score = score;
      best_cloud = cloud_in_world_frame_pixelized;
      best_pose = new_pose_estimate_neighbor;
    }
  }
  return std::make_tuple(best_pose, best_score, best_cloud);
}

// search for nearest p_hit for each beam end point. That's a looser restraint
// than p_hit and p_free
// score now is a negative number
double DreamGMapper::observation_model_score(
    PclCloudPtr cloud_in_world_frame_pixelized, const Pose2D &pose_estimate,
    const PointAccumulator &laser_point_accumulation_map) {

  double score = 0.0;
  double resolution_squared = resolution_ * resolution_;
  //   double gaussian_coefficient =
  //       1.0 / std::sqrt(2 * M_PI * beam_noise_variance_);
  Pixel2DWithCount pose_estimated_pixelized =
      Pixel2DWithCount(pose_estimate, resolution_);

  // each individual beam
  for (unsigned int i = 0; i < cloud_in_world_frame_pixelized->points.size();
       ++i) {
    auto endpoint = cloud_in_world_frame_pixelized
                        ->points[i]; // x, y in world frame, pixelized
    auto endpoint_pixel = Pixel2DWithCount(endpoint.x, endpoint.y);
    auto p_free_offset = SimpleRoboticsCppUtils::get_unit_vector_endpoint_pixel(
        endpoint_pixel, pose_estimated_pixelized);
    // Pre-compute the kernel. [0, 1, -1, 2, -2], use it in both xx and yy
    // directions. Find the pixelized p_hit given
    // We go over a 2D kernel layer by layer. Each layer goes around a square,
    // like [1, 0], [1,1], [1,-1], [] something like (2,4) before even going to
    char best_mu_squared = 10 * 10;
    bool match_found = false;
    for (const auto &xx : beam_search_kernel_) {
      for (const auto &yy : beam_search_kernel_) {
        auto p_hit = Pixel2DWithCount(endpoint.x + xx, endpoint.y + yy);
        auto p_free = Pixel2DWithCount(p_hit.x + p_free_offset.x,
                                       p_hit.y + p_free_offset.y);
        // // is_full returns If p_hit is full, then we have a match, then score
        // (2
        // // * 2000 lookups) and quit. false even on unknown pixels
        if (!laser_point_accumulation_map.contains(p_hit)) {
          continue;
        }
        // what if p_free is unknown? in ros gmapping unknown is equivalent to
        // free
        if ((laser_point_accumulation_map.is_full(
                p_hit, occupied_fullness_threshold_)) &&
            (!laser_point_accumulation_map.is_full(
                p_free, occupied_fullness_threshold_) && laser_point_accumulation_map.contains(p_free))) {
          char mu_squared = xx * xx + yy * yy;
          if (mu_squared < best_mu_squared)
            best_mu_squared = mu_squared;
          match_found = true;
        }
      }
    }

    if (match_found) {
      // compute the log score of the single beam match
      score += std::exp(-best_mu_squared * resolution_squared /
                        beam_noise_variance_);
    }
  }
  //   take exponential of the sum score
  return score; // score;
}

void DreamGMapper::normalize_weights(std::vector<Particle> &particles) {
  double sum = std::accumulate(
      particles.begin(), particles.end(), 0.0,
      [](double sum, const Particle &P) { return sum + P.weight_; });
  if (std::abs(sum) < 1e-60) {
    std::cout << "normalize_weights: sum is too close to 0: " << sum
              << std::endl;
    return;
  } else {
    std::for_each(particles.begin(), particles.end(),
                  [sum](Particle &p) { p.weight_ = p.weight_ / sum; });
  }
}

std::vector<unsigned int> DreamGMapper::get_resampled_indices(
    const std::vector<Particle> &particles) const {
  std::vector<unsigned int> resampled_indices(particle_num_, 0);
  std::vector<double> weights(particle_num_, 0.0);
  std::transform(particles.begin(), particles.end(), weights.begin(),
                 [](const Particle &p) -> double { return p.weight_; });
  std::mt19937 rng(std::random_device{}());
  // [0,1,2,...n-1]
  std::discrete_distribution<unsigned int> dist(weights.begin(), weights.end());
  for (auto &index : resampled_indices) {
    index = dist(rng);
  }
  return resampled_indices;
}

void DreamGMapper::add_scan_msg_to_map(Particle &p,
                                       const ScanMsgPtr &scan_msg) {
  // TODO: this can be done outside
  PclCloudPtr cloud_in_body_frame_full(new pcl::PointCloud<pcl::PointXYZ>());
  bool filling_success = DreamGMapping::fill_point_cloud(
      scan_msg, cloud_in_body_frame_full, false);
  if (!filling_success) {
    ROS_WARN("Failed to fill point cloud");
    return;
  }
  cloud_in_body_frame_full = DreamGMapping::transform_point_cloud_eigen4d(
      base_to_scan_, cloud_in_body_frame_full);
  auto cloud_in_world_frame_full = DreamGMapping::transform_point_cloud(
      p.pose_traj_.back()->to_se3(), cloud_in_body_frame_full);
  DreamGMapping::pixelize_point_cloud(cloud_in_world_frame_full, resolution_);
  auto pose_pixelized = SimpleRoboticsCppUtils::Pixel2DWithCount(
      *p.pose_traj_.back(), resolution_);

  for (unsigned int i = 0; i < scan_msg->ranges.size(); ++i) {
    const double range = scan_msg->ranges.at(i);
    if (range < scan_msg->range_min || range >= scan_msg->range_max)
      continue;
    // adding obstacle points only if its range is less than max
    const auto &endpoint = cloud_in_world_frame_full->points.at(i);
    const auto &endpoint_pixelized = SimpleRoboticsCppUtils::Pixel2DWithCount(
        SimpleRoboticsCppUtils::Pixel2DWithCount(endpoint.x, endpoint.y));
    if (scan_msg->range_max - range > resolution_) {

      p.laser_point_accumulation_map_.add_point(endpoint_pixelized.x,
                                                endpoint_pixelized.y, true);
    }
    auto line = SimpleRoboticsCppUtils::bresenham_rico_line(pose_pixelized,
                                                            endpoint_pixelized);
    for (unsigned int i = 0; i < line.size() - 1; ++i) {
      const auto &p_free = line[i];
      p.laser_point_accumulation_map_.add_point(p_free.x, p_free.y, false);
    }
  }
}

void DreamGMapper::
    resample_if_needed_and_update_particle_map_and_find_best_pose(
        const ScanMsgPtr &scan_msg) {
  normalize_weights(particles_);
  double normal_sqred_sum =
      std::accumulate(particles_.begin(), particles_.end(), 0.0,
                      [](double neff, const Particle &p) {
                        return neff + p.weight_ * p.weight_;
                      });
  double neff = 1.0 / normal_sqred_sum;
  // TODO
  std::cout << "normalized weights: " << std::endl;
  for (auto &p : particles_) {
    std::cout << p.weight_ << std::endl;
  }
  // TODO
  std::cout << "neff: " << neff << std::endl;

  // neff is maximal for equal weights. resample if neff < neff_threshold
  if (neff < particle_num_ / 2.0) {
    // TODO
    std::cout << "resampling" << std::endl;
    auto resampled_indices = get_resampled_indices(particles_);
    std::unordered_map<unsigned int, unsigned int> counts;
    std::vector<Particle> tmp_particles;
    tmp_particles.reserve(particle_num_);
    const double weight = 1.0 / particle_num_;
    for (unsigned int i = 0; i < particle_num_; ++i) {
      const unsigned int resampled_index = resampled_indices.at(i);
      ++counts[resampled_index];
      tmp_particles.push_back(particles_.at(resampled_index));
      tmp_particles.back().weight_ = weight;
      add_scan_msg_to_map(tmp_particles.back(), scan_msg);
    }
    particles_ = tmp_particles;

    find_most_weighted_particle_index(counts, best_particle_index_);
  } else {
    // just add point clouds into maps.
    for (unsigned int i = 0; i < particle_num_; ++i) {
      add_scan_msg_to_map(particles_.at(i), scan_msg);
    }
    find_most_weighted_particle_index(particles_, best_particle_index_);
  }
  std::vector<double> scores(particle_num_, 0);
  std::transform(particles_.begin(), particles_.end(), scores.begin(),
                 [](const Particle &particle) { return particle.weight_; });
}

// Our map is a fixed size one, sizing-up is currently not supported
void DreamGMapper::publish_map() {
  // get the best pose
  const auto &best_particle = particles_.at(best_particle_index_);
  map_.header.stamp = ros::Time::now();
  best_particle.laser_point_accumulation_map_.fill_ros_map(map_.data, map_size_,
                                                           origin_offset_);
  map_pub_.publish(map_);
  const auto &map_to_baselink = best_particle.pose_traj_.back()->to_se3();
  const Eigen::Matrix4d map_to_odom =
      map_to_baselink * last_odom_pose_.inverse();

  const Eigen::Isometry3d map_to_odom_iso(map_to_odom);
  geometry_msgs::TransformStamped tmp_tf_ =
      tf2::eigenToTransform(map_to_odom_iso);
  map_mutex_.lock();
  map_to_odom_tf_.transform = tmp_tf_.transform;
  map_mutex_.unlock();
  last_map_update_ = ros::Time::now();
}

void DreamGMapper::publish_tf() {
  map_mutex_.lock();
  map_to_odom_tf_.header.stamp = ros::Time::now();
  br_.sendTransform(map_to_odom_tf_);
  map_mutex_.unlock();
}
} // namespace DreamGMapping
