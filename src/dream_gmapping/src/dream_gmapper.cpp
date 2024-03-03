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
#include "nav_msgs/OccupancyGrid.h"
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
  nh_.getParam("wheel_dist", wheel_dist_);
  nh_.getParam("angular_active_threshold", angular_active_threshold_);
  nh_.getParam("translation_active_threshold", translation_active_threshold_);

  double d_v_std_dev, d_theta_std_dev;
  nh_.getParam("d_v_std_dev", d_v_std_dev);
  nh_.getParam("d_theta_std_dev", d_theta_std_dev);
  motion_covariances_ << d_v_std_dev, 0.0, 0.0, d_theta_std_dev;
  nh_.getParam("resolution", resolution_);
  nh_.getParam("beam_noise_sigma_squared", beam_noise_variance_);
  nh_.getParam("beam_kernel_size", beam_kernel_size_);
  log_prob_beam_not_found_in_kernel_ =
      -(2 * std::pow(resolution_ * beam_kernel_size_, 2)) /
      beam_noise_variance_;
  double map_size_in_meters;
  nh_.getParam("map_size_in_meters", map_size_in_meters);
  map_size_ = static_cast<unsigned int>(map_size_in_meters / resolution_);
  map_pub_ = nh_.advertise<nav_msgs::OccupancyGrid>("map", 1);
  initialize_map();

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

  initialize_motion_set();
  for (char i = 1; i < beam_kernel_size_; ++i) {
    beam_search_kernel_[2 * i - 1] = i;
    beam_search_kernel_[2 * i] = -i;
  }

  ROS_INFO_STREAM("Successfully initialized dream_gmapping");
}

DreamGMapper::~DreamGMapper() = default;

void DreamGMapper::initialize_map() {
  map_.header.frame_id = "map";
  map_.info.resolution = resolution_;
  map_.info.width = map_size_;
  map_.info.height = map_size_;
  // this is the real world pose of (0, 0) of the map.
  map_.info.origin.position.x = -map_size_ / 2 * resolution_;
  map_.info.origin.position.y = -map_size_ / 2 * resolution_;
  map_.info.origin.position.z = 0.0;
  origin_offset_ = map_size_ * map_size_ / 2;
  map_.info.origin.orientation.w = 1.0;
}

void DreamGMapper::initialize_motion_set() {
  // TODO - to move
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

// TODO test
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
      // Unit-testble optimization function
      auto [s, m, cloud_in_world_frame] =
          optimizeAfterIcp(particle, T_icp_output, scan_msg);
    } else {
      // draw from motion model
      auto [s, m] = SimpleRoboticsCppUtils::draw_from_icc(
          *(particle.pose_traj_.back()), screw_displacement,
          motion_covariances_);
      score = s;
      new_pose_estimate = m;
      DreamGMapping::get_point_cloud_in_world_frame(new_pose_estimate,
                                                    cloud_in_world_frame);
      // TODO: pixelize
      DreamGMapping::pixelize_point_cloud(cloud_in_world_frame, resolution_);
    }
    // store score, and the new pose now, new point cloud in world frame
    cloud_in_world_frame_vec.push_back(cloud_in_world_frame);
    particle.pose_traj_.back() =
        std::make_shared<SimpleRoboticsCppUtils::Pose2D>(new_pose_estimate);
    particle.weight_ = score;
  }

  resample_if_needed_and_update_particle_map_and_find_best_pose(
      cloud_in_world_frame_vec);
  publish_map();
  store_last_scan(cloud);
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

// TODO test 2
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
    DreamGMapping::fill_point_cloud(scan_msg, cloud_in_world_frame_pixelized);
    DreamGMapping::get_point_cloud_in_world_frame(
        new_pose_estimate_neighbor, cloud_in_world_frame_pixelized);
    DreamGMapping::pixelize_point_cloud(cloud_in_world_frame_pixelized,
                                        resolution_);
    // observation model
    double score = observation_model_score(
        cloud_in_world_frame_pixelized, scan_msg, new_pose_estimate_neighbor,
        particle.laser_point_accumulation_map_);
    if (score > best_score) {
      best_score = score;
      best_cloud = cloud_in_world_frame_pixelized;
      best_pose = new_pose_estimate_neighbor;
    }
  }

  return std::make_tuple(best_pose, best_score, best_cloud);
}

// TODO test 1
double DreamGMapper::observation_model_score(
    PclCloudPtr cloud_in_world_frame_pixelized, ScanMsgPtr scan_msg,
    const Pose2D &pose_estimate,
    const PointAccumulator &laser_point_accumulation_map) {
  double score = 0;
  Pixel2DWithCount pose_estimated_pixelized =
      Pixel2DWithCount(pose_estimate, resolution_);

  // go over all scan message, and their angle
  for (double angle = scan_msg->angle_min, i = 0; i < scan_msg->ranges.size();
       i++, angle += scan_msg->angle_increment) {
    auto endpoint = cloud_in_world_frame_pixelized
                        ->points[i]; // x, y in world frame, pixelized
    auto endpoint_pixel = Pixel2DWithCount(endpoint.x, endpoint.y);
    auto p_free_offset = SimpleRoboticsCppUtils::get_unit_vector_endpoint_pixel(
        endpoint_pixel, pose_estimated_pixelized);
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
        // TODO to remove after testing
        std::cout << "xx: " << xx << "yy: " << yy << std::endl;
        // find p_hit and p_free right in front of it.
        auto p_hit = Pixel2DWithCount(endpoint.x + xx, endpoint.y + yy);
        auto p_free = Pixel2DWithCount(p_hit.x + p_free_offset.x,
                                       p_hit.y + p_free_offset.y);
        // If p_hit is full or p_hit is a max, but p_free is not, then we have a
        // match, then score (2 * 2000 lookups) and quit. TODO: is_full returns
        // false even on unknown pixels
        if (!laser_point_accumulation_map.is_full(p_free) &&
            (laser_point_accumulation_map.is_full(p_hit) ||
             scan_msg->ranges[i] == scan_msg->range_max)) {
          // compute the log score of the single beam match
          score +=
              -(xx * xx + yy * yy) * resolution_squared / beam_noise_variance_;
          break;
        }
      }
    }
    // If not found, add multiply exp(-1*kernel_size_^2/sigma), PRECOMPUTED
    if (!match_found) {
      score += log_prob_beam_not_found_in_kernel_;
    }
  }

  // take exponential of the sum score
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
  for (const auto &point : cloud_in_world_frame_vec_pixelized->points) {
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

void DreamGMapper::
    resample_if_needed_and_update_particle_map_and_find_best_pose(
        const std::vector<PclCloudPtr> &cloud_in_world_frame_vec) {
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
      const unsigned int resampled_index = resampled_indices[i];
      ++counts[resampled_index];
      tmp_particles.push_back(particles_.at(resampled_index));
      tmp_particles.back().weight_ = weight;
      add_cloud_in_world_frame_to_map(tmp_particles.back(),
                                      cloud_in_world_frame_vec[i]);
    }
    particles_ = tmp_particles;

    find_most_weighted_particle_index(counts, best_particle_index_);
  } else {
    // just add point clouds into maps.
    for (unsigned int i = 0; i < particle_num_; ++i) {
      add_cloud_in_world_frame_to_map(particles_[i],
                                      cloud_in_world_frame_vec[i]);
    }
    find_most_weighted_particle_index(particles_, best_particle_index_);
  }
  std::vector<double> scores(particle_num_, 0);
  std::transform(particles_.begin(), particles_.end(), scores.begin(),
                 [](const Particle &particle) { return particle.weight_; });
}

// TODO
// Our map is a fixed size one, sizing-up is currently not supported
void DreamGMapper::publish_map() {
  // get the best pose
  const auto &best_particle = particles_.at(best_particle_index_);
  map_.data = std::vector<int8_t>(map_size_ * map_size_, -1);
  map_.header.stamp = ros::Time::now();
  best_particle.laser_point_accumulation_map_.fill_ros_map(map_.data, map_size_,
                                                           origin_offset_);
  map_pub_.publish(map_);
}

} // namespace DreamGMapping
