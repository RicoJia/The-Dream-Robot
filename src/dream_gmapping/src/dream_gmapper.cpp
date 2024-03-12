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
#include "dream_gmapping/dream_gmapping_utils.hpp"
#include "geometry_msgs/TransformStamped.h"
#include "nav_msgs/OccupancyGrid.h"
#include "ros/duration.h"
#include "simple_robotics_cpp_utils/math_utils.hpp"
#include "tf2/exceptions.h"
#include <algorithm>
#include <cmath>
#include <iostream>
#include <limits>
#include <memory>
#include <numeric>
#include <pcl/common/transforms.h>
#include <random>
#include <ros/node_handle.h>
#include <ros/ros.h>
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
  nh_.getParam("beam_kernel_size", beam_kernel_size_);
  nh_.getParam("skip_invalid_beams", skip_invalid_beams_);

  log_prob_beam_not_found_in_kernel_ =
      -(2 * std::pow(resolution_ * beam_kernel_size_, 2)) /
      beam_noise_variance_;
  double map_size_in_meters;
  nh_.getParam("map_size_in_meters", map_size_in_meters);
  map_size_ = static_cast<unsigned int>(map_size_in_meters / resolution_);
  if (map_size_ % 2 == 0) {
    map_size_++;
    ROS_WARN_STREAM(
        "To have an odd number of cells in the map, map size now is "
        << map_size_ * resolution_);
  }
  map_pub_ = nh_.advertise<nav_msgs::OccupancyGrid>("map", 1);
  initialize_map();

  //   DreamGMapping::print_all_nodehandle_params(nh_);
  ROS_INFO_STREAM("Successfully read parameters for dream_gmapping");

  // we are not using TimeSynchronizer because tf2 already provides buffering
  // with timestamps
  laser_sub_ = nh_.subscribe("scan", 1, &DreamGMapper::laser_scan, this);

  // std::shared_ptr<Rigid2D::>
  for (unsigned int i = 0; i < particle_num_; i++) {
    // TODO: check if weight is 1/particle_num;
    particles_.push_back(Particle{
        1.0 / particle_num_,
        {std::make_shared<SimpleRoboticsCppUtils::Pose2D>(0.0, 0.0, 0.0)}});
  }

  initialize_motion_set();
  for (char i = 1; i < beam_kernel_size_; ++i) {
    beam_search_kernel_[2 * i - 1] = i;
    beam_search_kernel_[2 * i] = -i;
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
}

void DreamGMapper::initialize_motion_set() {
  motion_set_ = std::vector<Eigen::Matrix4d>{
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
}

void DreamGMapper::laser_scan(
    const boost::shared_ptr<const sensor_msgs::LaserScan> &scan_msg) {
  // - wait for the first scan message, get tf;

  geometry_msgs::TransformStamped odom_to_base;
  try {
    odom_to_base =
        tf_buffer_.lookupTransform(odom_frame_, base_frame_, ros::Time(0));
    auto received_time = (ros::Time(0) - odom_to_base.header.stamp).toSec();
    if (received_time > 1.0) {
      ROS_WARN_STREAM("Odom to base transform was received " << received_time);
      return;
    }
  } catch (tf2::TransformException &e) {
    ROS_WARN("%s", e.what());
    return;
  }

  if (!received_first_laser_scan_) {
    // Store the laser->base transform
    received_first_laser_scan_ = true;
    store_last_scan(scan_msg, skip_invalid_beams_);
    ROS_DEBUG_STREAM("Received first laser scan");
    // We are not storing the odom here, because that will be used for icp next
    return;
  }

  // odom_to_base to eigen -> calculate T_delta -> store in last_odom_pose
  Eigen::Affine3d transform_eigen =
      tf2::transformToEigen(odom_to_base.transform);
  Eigen::Matrix4d current_odom_pose = transform_eigen.matrix();
  Eigen::Matrix4d T_delta = last_odom_pose_.inverse() * current_odom_pose;
  // T_delta -> delta_translation, delta_rotation
  Eigen::Vector3d delta_translation_vec = T_delta.block<3, 1>(0, 3);
  double delta_translation = delta_translation_vec.norm();
  double delta_theta =
      atan2(T_delta(1, 0), T_delta(0, 0)); // Rotation around Z-axis
  last_odom_pose_ = current_odom_pose;

  // If odom, angular distance is not enough, skip.
  if (delta_translation < translation_active_threshold_ &&
      delta_theta < angular_active_threshold_)
    return;

  PclCloudPtr cloud_in_body_frame(new pcl::PointCloud<pcl::PointXYZ>());
  bool filling_success = DreamGMapping::fill_point_cloud(
      scan_msg, cloud_in_body_frame, skip_invalid_beams_);
  if (!filling_success) {
    ROS_WARN("Failed to fill point cloud");
    return;
  };

  // - icp: scan match, get initial guess
  Eigen::Matrix4d T_icp_output = Eigen::Matrix4d::Identity();
  bool icp_converge = scan_msg->ranges.size() != 0 &&
                      DreamGMapping::icp_2d(last_cloud_, cloud_in_body_frame,
                                            T_delta, T_icp_output);

  std::vector<PclCloudPtr> cloud_in_world_frame_vec{};
  cloud_in_world_frame_vec.reserve(particle_num_);
  // TODO
  std::cout << "icp output: " << T_icp_output << std::endl;
  // TODO
  std::cout << "current_odom_pose" << current_odom_pose << std::endl;

  for (auto &particle : particles_) {
    PclCloudPtr cloud_in_world_frame(new pcl::PointCloud<pcl::PointXYZ>());
    double score;
    SimpleRoboticsCppUtils::Pose2D new_pose_estimate{0, 0, 0};
    // In the if clause, we need: score, the new pose and the new point cloud in
    // world frame
    if (icp_converge) {
      // Unit-testble optimization function
      auto [m, s, c] = optimize_after_icp(particle, T_icp_output, scan_msg);
      score = s;
      new_pose_estimate = m;
      cloud_in_world_frame = c;
    } else {
      // Update with odom
      score = particle.weight_;
      // TODO: do we need to add noise here??
      new_pose_estimate = SimpleRoboticsCppUtils::Pose2D(
          particle.pose_traj_.back()->to_se3() * T_delta);
      cloud_in_world_frame = DreamGMapping::get_point_cloud_in_world_frame(
          new_pose_estimate, cloud_in_body_frame);
      DreamGMapping::pixelize_point_cloud(cloud_in_world_frame, resolution_);
    }

    // store score, and the new pose now, new point cloud in world frame
    cloud_in_world_frame_vec.push_back(cloud_in_world_frame);
    particle.pose_traj_.back() =
        std::make_shared<SimpleRoboticsCppUtils::Pose2D>(new_pose_estimate);
    particle.weight_ = score;
  }

  resample_if_needed_and_update_particle_map_and_find_best_pose(scan_msg);
  publish_map_and_tf();
  store_last_scan(cloud_in_body_frame);
}

void DreamGMapper::store_last_scan(
    const boost::shared_ptr<const sensor_msgs::LaserScan> &scan_msg,
    const bool &skip_invalid_beams) {
  DreamGMapping::fill_point_cloud(scan_msg, last_cloud_, skip_invalid_beams);
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
DreamGMapper::optimize_after_icp(const DreamGMapping::Particle &particle,
                                 const Eigen::Ref<Eigen::Matrix4d> T_icp_output,
                                 ScanMsgPtr scan_msg) {
  Eigen::Matrix4d new_pose_estimate =
      T_icp_output * (particle.pose_traj_.back()->to_se3());
  double score = 0.0;
  PclCloudPtr best_cloud(new pcl::PointCloud<pcl::PointXYZ>());
  double best_score = std::numeric_limits<double>::min();
  Pose2D best_pose(0, 0, 0);
  for (const auto &motion : motion_set_) {
    // search around new_pose_estimate, each neighbor has a pixelized pointcloud
    auto new_pose_estimate_neighbor =
        SimpleRoboticsCppUtils::Pose2D(motion * new_pose_estimate);
    PclCloudPtr cloud_in_world_frame_pixelized(
        new pcl::PointCloud<pcl::PointXYZ>());
    // we are comparing the valid point cloud
    DreamGMapping::fill_point_cloud(scan_msg, cloud_in_world_frame_pixelized,
                                    skip_invalid_beams_);
    cloud_in_world_frame_pixelized =
        DreamGMapping::get_point_cloud_in_world_frame(
            new_pose_estimate_neighbor, cloud_in_world_frame_pixelized);
    DreamGMapping::pixelize_point_cloud(cloud_in_world_frame_pixelized,
                                        resolution_);
    // observation model
    double score = observation_model_score(
        cloud_in_world_frame_pixelized, new_pose_estimate_neighbor,
        particle.laser_point_accumulation_map_);
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
double DreamGMapper::observation_model_score(
    PclCloudPtr cloud_in_world_frame_pixelized, const Pose2D &pose_estimate,
    const PointAccumulator &laser_point_accumulation_map) {
  double score = 0;
  Pixel2DWithCount pose_estimated_pixelized =
      Pixel2DWithCount(pose_estimate, resolution_);
  for (unsigned int i = 0; i < cloud_in_world_frame_pixelized->points.size();
       ++i) {
    auto endpoint = cloud_in_world_frame_pixelized
                        ->points[i]; // x, y in world frame, pixelized
    auto endpoint_pixel = Pixel2DWithCount(endpoint.x, endpoint.y);
    // Pre-compute the kernel. [0, 1, -1, 2, -2], use it in both xx and yy
    // directions. Find the pixelized p_hit given
    // We go over a 2D kernel layer by layer. Each layer goes around a square,
    // like [1, 0], [1,1], [1,-1], [] something like (2,4) before even going to
    bool match_found = false;
    double resolution_squared = resolution_ * resolution_;
    for (const char &xx : beam_search_kernel_) {
      for (unsigned int i = 0;
           std::abs(beam_search_kernel_.at(i)) < std::abs(xx) &&
           i < beam_search_kernel_.size();
           i++) {
        const char &yy = beam_search_kernel_[i];
        // find p_hit and p_free right in front of it.
        auto p_hit = Pixel2DWithCount(endpoint.x + xx, endpoint.y + yy);
        // what do you do when the point is unknown yet - just score?  TODO:
        // is_full returns If p_hit is full, then we have a match, then score (2
        // * 2000 lookups) and quit. false even on unknown pixels
        if (laser_point_accumulation_map.is_full(p_hit)) {
          // compute the log score of the single beam match
          score +=
              -(xx * xx + yy * yy) * resolution_squared / beam_noise_variance_;
          match_found = true;
          break;
        }
      }
    }
    // std::cout<<"observation_model_score, after kernel search:
    // log"<<score<<std::endl; If not found, add multiply
    // exp(-1*kernel_size_^2/sigma), PRECOMPUTED
    if (!match_found) {
      score += log_prob_beam_not_found_in_kernel_;
      //   //TODO
      //   std::cout<<"observation_model_score, not found:
      //   log"<<score<<std::endl;
    }
  }
  //   take exponential of the sum score
  return std::exp(score);
}

void DreamGMapper::normalize_weights(std::vector<Particle> &particles) {
  double sum = std::accumulate(
      particles.begin(), particles.end(), 0.0,
      [](double sum, const Particle &P) { return sum + P.weight_; });
  if (std::abs(sum) < 1e-5) {
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

void DreamGMapper::add_cloud_in_world_frame_to_map(
    Particle &p, const PclCloudPtr &cloud_in_world_frame_vec_pixelized) {
  for (unsigned int i = 0;
       i < cloud_in_world_frame_vec_pixelized->points.size(); ++i) {
    const auto &point = cloud_in_world_frame_vec_pixelized->points.at(i);
    p.laser_point_accumulation_map_.add_point(point.x, point.y, true);
    auto pose_pixelized = SimpleRoboticsCppUtils::Pixel2DWithCount(
        *p.pose_traj_.back(), resolution_);
    auto endpoint_pixelized = SimpleRoboticsCppUtils::Pixel2DWithCount(
        SimpleRoboticsCppUtils::Pixel2DWithCount(point.x, point.y));
    // Need to add all free points as well
    auto line = SimpleRoboticsCppUtils::bresenham_rico_line(pose_pixelized,
                                                            endpoint_pixelized);
    for (unsigned int i = 0; i < line.size() - 1; ++i) {
      const auto &p_free = line[i];
      p.laser_point_accumulation_map_.add_point(p_free.x, p_free.y, false);
    }
  }
}

void DreamGMapper::add_scan_msg_to_map(Particle &p,
                                       const ScanMsgPtr &scan_msg) {
  PclCloudPtr cloud_in_body_frame_full(new pcl::PointCloud<pcl::PointXYZ>());
  bool filling_success = DreamGMapping::fill_point_cloud(
      scan_msg, cloud_in_body_frame_full, false);
  if (!filling_success) {
    ROS_WARN("Failed to fill point cloud");
    return;
  }
  auto cloud_in_world_frame_full =
      DreamGMapping::get_point_cloud_in_world_frame(
          p.pose_traj_.back()->to_se3(), cloud_in_body_frame_full);
  DreamGMapping::pixelize_point_cloud(cloud_in_world_frame_full, resolution_);

  for (unsigned int i = 0; i < scan_msg->ranges.size(); ++i) {
    const double range = scan_msg->ranges.at(i);
    if (range < scan_msg->range_min)
      continue;
    // adding obstacle points only if its range is less than max
    const auto &endpoint = cloud_in_world_frame_full->points.at(i);
    const auto &endpoint_pixelized = SimpleRoboticsCppUtils::Pixel2DWithCount(
        SimpleRoboticsCppUtils::Pixel2DWithCount(endpoint.x, endpoint.y));
    if (std::abs(range - scan_msg->range_max) > resolution_) {

      p.laser_point_accumulation_map_.add_point(endpoint_pixelized.x,
                                                endpoint_pixelized.y, true);
    }
    auto pose_pixelized = SimpleRoboticsCppUtils::Pixel2DWithCount(
        *p.pose_traj_.back(), resolution_);
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
  // neff is maximal for equal weights. resample if neff < neff_threshold
  if (neff < particle_num_ / 2.0) {
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
void DreamGMapper::publish_map_and_tf() {
  // get the best pose
  const auto &best_particle = particles_.at(best_particle_index_);
  map_.header.stamp = ros::Time::now();
  best_particle.laser_point_accumulation_map_.fill_ros_map(map_.data, map_size_,
                                                           origin_offset_);
  map_pub_.publish(map_);
  const auto &map_to_baselink = best_particle.pose_traj_.back()->to_se3();
  const Eigen::Matrix4d map_to_odom =
      map_to_baselink * last_odom_pose_.inverse();
  // TODO
  std::cout << "map to baselink: " << map_to_baselink << std::endl;
  // TODO
  std::cout << "last_odom_pose: " << last_odom_pose_ << std::endl;
  // TODO
  std::cout << "map to odom: " << map_to_odom << std::endl;

  const Eigen::Isometry3d map_to_odom_iso(map_to_odom);
  map_to_odom_tf_.header.stamp = ros::Time::now();
  geometry_msgs::TransformStamped tmp_tf_ =
      tf2::eigenToTransform(map_to_odom_iso);
  map_to_odom_tf_.transform = tmp_tf_.transform;
  br_.sendTransform(map_to_odom_tf_);
}

/**
 * ********************************************************************************
 * GRAVEYARD
 * ********************************************************************************
 */
// Problem with this method is: cloud_in_world_frame_pixelized does not have the
// same number of elements as scan_msg double
// DreamGMapper::observation_model_score(
//     PclCloudPtr cloud_in_world_frame_pixelized, ScanMsgPtr scan_msg,
//     const Pose2D &pose_estimate,
//     const PointAccumulator &laser_point_accumulation_map) {
//   double score = 0;
//   Pixel2DWithCount pose_estimated_pixelized =
//       Pixel2DWithCount(pose_estimate, resolution_);

//   // go over all scan message, and their angle
//   for (double angle = scan_msg->angle_min, i = 0; i <
//   scan_msg->ranges.size();
//        i++, angle += scan_msg->angle_increment) {
//     auto endpoint = cloud_in_world_frame_pixelized
//                         ->points[i]; // x, y in world frame, pixelized
//     auto endpoint_pixel = Pixel2DWithCount(endpoint.x, endpoint.y);
//     auto p_free_offset =
//     SimpleRoboticsCppUtils::get_unit_vector_endpoint_pixel(
//         endpoint_pixel, pose_estimated_pixelized);
//     // Pre-compute the kernel. [0, 1, -1, 2, -2], use it in both xx and yy
//     // directions. Find the pixelized p_hit given
//     // We go over a 2D kernel layer by layer. Each layer goes around a
//     square,
//     // like [1, 0], [1,1], [1,-1], [] something like (2,4) before even going
//     to bool match_found = false; double resolution_squared = resolution_ *
//     resolution_; for (const char &xx : beam_search_kernel_) {
//       for (unsigned int i = 0;
//            std::abs(beam_search_kernel_.at(i)) < std::abs(xx) &&
//            i < beam_search_kernel_.size();
//            i++) {
//         const char &yy = beam_search_kernel_[i];
//         // find p_hit and p_free right in front of it.
//         auto p_hit = Pixel2DWithCount(endpoint.x + xx, endpoint.y + yy);
//         auto p_free = Pixel2DWithCount(p_hit.x + p_free_offset.x,
//                                        p_hit.y + p_free_offset.y);
//         // what do you do when the point is unknown yet - just score?
//         // If p_hit is full or p_hit is a max, but p_free is not, then we
//         have a
//         // match, then score (2 * 2000 lookups) and quit. TODO: is_full
//         returns
//         // false even on unknown pixels
//         if (!laser_point_accumulation_map.is_full(p_free) &&
//             (laser_point_accumulation_map.is_full(p_hit) ||
//              scan_msg->ranges[i] == scan_msg->range_max)) {
//           // compute the log score of the single beam match
//           score +=
//               -(xx * xx + yy * yy) * resolution_squared /
//               beam_noise_variance_;
//           match_found = true;
//           break;
//         }
//       }
//     }
//     // TODO
//     // std::cout<<"observation_model_score, after kernel search:
//     // log"<<score<<std::endl; If not found, add multiply
//     // exp(-1*kernel_size_^2/sigma), PRECOMPUTED
//     if (!match_found) {
//       score += log_prob_beam_not_found_in_kernel_;
//       //   //TODO
//       //   std::cout<<"observation_model_score, not found:
//       //   log"<<score<<std::endl;
//     }
//   }

//   // take exponential of the sum score
//   return std::exp(score);
// }
} // namespace DreamGMapping
