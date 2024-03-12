#pragma once
#include "dream_gmapping/dream_gmapping_utils.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include <cmath>
#include <geometry_msgs/TransformStamped.h>
#include <memory>
#include <message_filters/subscriber.h>
#include <ros/node_handle.h>
#include <ros/ros.h>
#include <simple_robotics_cpp_utils/math_utils.hpp>
#include <simple_robotics_cpp_utils/rigid2d.hpp>
#include <tf2_ros/message_filter.h>
#include <tf2_ros/transform_listener.h>
#include <vector>

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

protected:
  // configurable parameters
  std::string base_frame_ = "base_link";
  std::string map_frame_ = "map";
  std::string odom_frame_ = "odom";
  double map_update_interval_ = 0.1; // 10 hz
  double max_range_;
  int particle_num_ = 50;
  double angular_active_threshold_ = M_PI / 6.0;
  double translation_active_threshold_ = 0.2;
  Eigen::Matrix2d motion_covariances_{Eigen::Matrix2d::Identity()};
  // No need to be user-initialized
  Eigen::Vector3d motion_means_{Eigen::Vector3d::Zero()};
  double resolution_;
  double beam_noise_variance_;
  double log_prob_beam_not_found_in_kernel_;
  double beam_kernel_size_;
  bool skip_invalid_beams_ = true;

  // inconfigurable parameters
  // no need to store
  bool received_first_laser_scan_ = false;
  ros::Subscriber laser_sub_;
  tf2_ros::Buffer tf_buffer_ = tf2_ros::Buffer();
  tf2_ros::TransformListener tf_listener_ =
      tf2_ros::TransformListener(tf_buffer_);
  tf2_ros::TransformBroadcaster br_;
  geometry_msgs::TransformStamped map_to_odom_tf_;

  // TODO: to demolish
  Eigen::Matrix4d last_odom_pose_ = Eigen::Matrix4d::Identity();

  PclCloudPtr last_cloud_{new pcl::PointCloud<pcl::PointXYZ>()};
  std::vector<DreamGMapping::Particle> particles_;
  // transforms to neighbors around a pose estimate
  std::vector<Eigen::Matrix4d> neighbor_transforms_;
  std::vector<Eigen::Matrix4d> motion_set_;
  unsigned int best_particle_index_ = 0;
  std::vector<char> beam_search_kernel_ =
      std::vector<char>(2 * beam_kernel_size_ + 1, 0);

  unsigned int map_size_;
  unsigned int origin_offset_;
  ros::Publisher map_pub_;
  nav_msgs::OccupancyGrid map_;

  // TODO: docstrings

  void initialize_map();

  void initialize_motion_set();

  // get the most recent odom -> draw a new noise -> go through all particles,
  void store_last_scan(
      const boost::shared_ptr<const sensor_msgs::LaserScan> &scan_msg,
      const bool &skip_invalid_beams);
  void store_last_scan(PclCloudPtr to_update);

  std::tuple<SimpleRoboticsCppUtils::Pose2D, double, PclCloudPtr>
  optimize_after_icp(const DreamGMapping::Particle &particle,
                     const Eigen::Ref<Eigen::Matrix4d> T_icp_output,
                     ScanMsgPtr scan_msg);

  /**
   * @brief Score a point cloud in world frame pixels based on a gaussian
   * observation model
   *
   * @param cloud_in_world_frame_pixelized :  point cloud in world frame pixels
   * @param scan_msg : original point cloud that has range information
   * @param pose_estimate pose estimate of the robot
   * @return double: gaussian likelihood of having all beams
   */

  double
  observation_model_score(PclCloudPtr cloud_in_world_frame_pixelized,
                          const Pose2D &pose_estimate,
                          const PointAccumulator &laser_point_accumulation_map);

  void normalize_weights(std::vector<Particle> &particles);
  /**
   * @brief Return indices of particles that will be copied to the next round
   * (so that is resampling)
   *
   * @param particles
   * @return std::vector<unsigned int> Indices of particles in particles that
   * will be copied to the next round
   */
  std::vector<unsigned int>
  get_resampled_indices(const std::vector<Particle> &particles) const;

  void add_scan_msg_to_map(Particle &p, const ScanMsgPtr &scan_msg);
  void resample_if_needed_and_update_particle_map_and_find_best_pose(
      const ScanMsgPtr &scan_msg);
  void publish_map_and_tf();
};
} // namespace DreamGMapping