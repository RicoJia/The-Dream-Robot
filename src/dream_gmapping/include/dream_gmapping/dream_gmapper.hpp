#pragma once
#include "dream_gmapping/dream_gmapping_utils.hpp"
#include "ros/publisher.h"
#include "tf2_ros/transform_broadcaster.h"
#include <cmath>
#include <eigen3/Eigen/src/Core/Matrix.h>
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
  void publish_tf();

protected:
  // configurable parameters
  std::string base_frame_ = "base_link";
  std::string map_frame_ = "map";
  std::string odom_frame_ = "odom";
  std::string scan_frame_ = "scan";
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
  double prob_not_found_;
  double half_beam_kernel_size_;
  bool skip_invalid_beams_ = true;
  bool publish_debug_scan_ = false;
  ros::Publisher last_cloud_debug_scan_pub_;
  ros::Publisher current_cloud_debug_scan_pub_;
  double occupied_fullness_threshold_;

  // inconfigurable parameters
  // no need to store
  bool received_first_laser_scan_ = false;
  ros::Subscriber laser_sub_;
  tf2_ros::Buffer tf_buffer_ = tf2_ros::Buffer();
  tf2_ros::TransformListener tf_listener_ =
      tf2_ros::TransformListener(tf_buffer_);
  tf2_ros::TransformBroadcaster br_;
  geometry_msgs::TransformStamped map_to_odom_tf_;

  Eigen::Matrix4d last_odom_pose_ = Eigen::Matrix4d::Identity();
  Eigen::Matrix4d base_to_scan_ = Eigen::Matrix4d::Zero();

  PclCloudPtr last_cloud_{new pcl::PointCloud<pcl::PointXYZ>()};
  std::vector<DreamGMapping::Particle> particles_;
  // transforms to neighbors around a pose estimate
  std::vector<Eigen::Matrix4d> neighbor_transforms_;
  // TODO
  std::vector<Eigen::Matrix4d> motion_set_;
  std::vector<std::vector<Eigen::Matrix4d>> optimization_transforms_vec_;

  unsigned int best_particle_index_ = 0;
  std::vector<int> beam_search_kernel_;

  unsigned int map_size_;
  unsigned int origin_offset_;
  ros::Publisher map_pub_;
  nav_msgs::OccupancyGrid map_;
  ros::Time last_map_update_;
  std::mutex map_mutex_;

  // TODO: docstrings

  void initialize_map();

  void initialize_motion_set(const int &initialize_motion_set);

  bool initialize_base_to_scan();

  // get the most recent odom -> draw a new noise -> go through all particles,
  void store_last_scan(
      const boost::shared_ptr<const sensor_msgs::LaserScan> &scan_msg,
      const bool &skip_invalid_beams);
  void store_last_scan(PclCloudPtr to_update);

  void publish_debug_scans(PclCloudPtr last_cloud, PclCloudPtr current_cloud);

  bool get_odom_to_base(Eigen::Matrix4d &T_delta,
                        Eigen::Matrix4d &current_odom_pose);

  std::tuple<SimpleRoboticsCppUtils::Pose2D, double, PclCloudPtr>
  optimize_after_icp(const DreamGMapping::Particle &particle,
                     const Eigen::Ref<Eigen::Matrix4d> T_icp_output,
                     PclCloudPtr cloud_in_body_frame);

  std::tuple<SimpleRoboticsCppUtils::Pose2D, double, PclCloudPtr>
  gradient_descent_optimize(const DreamGMapping::Particle &particle,
                            const Eigen::Ref<Eigen::Matrix4d> T_icp_output,
                            PclCloudPtr cloud_in_body_frame);
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
  void publish_map();
};
} // namespace DreamGMapping

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
//       score += prob_not_found_;
//       //   //TODO
//       //   std::cout<<"observation_model_score, not found:
//       //   log"<<score<<std::endl;
//     }
//   }

//   // take exponential of the sum score
//   return std::exp(score);
// }

//   if (icp_converge) {
//     // TODO: to add this to non-convergence case
//     // add motion noise and update.
//     // This method takes in theta and translation. But it should be
//     real
//     // screw motion
//     auto [motion_score, new_motion_pose] =
//         SimpleRoboticsCppUtils::draw_from_icc(
//             *particle.pose_traj_.back(), {delta_translation,
//             delta_theta}, motion_covariances_);
//     *particle.pose_traj_.back() = new_motion_pose;
//     // Unit-testble optimization function
//     auto [m, s, c] =
//         optimize_after_icp(particle, T_icp_output,
//         cloud_in_body_frame);
//     score = s;
//     new_pose_estimate = m;
//     cloud_in_world_frame = c;
//     // TODO
//     std::cout << "score in icp case: " << score << std::endl;
//     std::cout << "icp corrected pose: " << std::endl
//               << new_pose_estimate << std::endl;
//   } else {
//     score = particle.weight_;
//     // TODO: do we need to add noise here??
//     new_pose_estimate = SimpleRoboticsCppUtils::Pose2D(
//         particle.pose_traj_.back()->to_se3() * T_delta);
//     // TODO: cloud_in_world_frame is not being used here
//     cloud_in_world_frame = DreamGMapping::transform_point_cloud(
//         new_pose_estimate, cloud_in_body_frame);
//     DreamGMapping::pixelize_point_cloud(cloud_in_world_frame,
//     resolution_);
//   }

// // - icp: scan match, get initial guess
// Eigen::Matrix4d T_icp_output = Eigen::Matrix4d::Identity();
// bool icp_converge = scan_msg->ranges.size() != 0 &&
//                     DreamGMapping::icp_2d(last_cloud_,
//                     cloud_in_body_frame,
//                                           T_delta, T_icp_output);
// publish_debug_scans(last_cloud_, cloud_in_body_frame);

// //TODO
// std::cout<<"current odom: "<<std::endl<<current_odom_pose<<std::endl;
// std::cout << "icp output: " << std::endl << T_icp_output << std::endl;