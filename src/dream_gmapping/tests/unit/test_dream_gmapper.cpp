#include "dream_gmapping/dream_gmapper.hpp"
#include "dream_gmapping/dream_gmapping_utils.hpp"
#include "dream_gmapping/dream_odometer.hpp"
#include "ros/duration.h"
#include "ros/init.h"
#include "ros/node_handle.h"
#include "ros/rate.h"
#include "sensor_msgs/LaserScan.h"
#include "shared_test_utils.hpp"
#include "simple_robotics_cpp_utils/performance_utils.hpp"
#include "std_msgs/Float32MultiArray.h"
#include <cmath>
#include <gtest/gtest.h>
#include <ostream>
#include <simple_robotics_cpp_utils/math_utils.hpp>
#include <simple_robotics_cpp_utils/rigid2d.hpp>
#include <unistd.h>
#include <unordered_map>
#include <utility>

using DreamGMapping::Particle;
/**
 * -----------------------------------------------------------------
 * Test Structs and Suite
 * -----------------------------------------------------------------
 */
struct TestableDreamOdometer : public DreamGMapping::DreamOdometer {
  ros::Publisher odom_pub_;
  double left_wheel_pos_ = 0.0;
  double right_wheel_pos_ = 0.0;

public:
  ~TestableDreamOdometer() { odom_pub_.shutdown(); }
  explicit TestableDreamOdometer(ros::NodeHandle nh) : DreamOdometer(nh) {
    odom_pub_ = nh.advertise<std_msgs::Float32MultiArray>(WHEEL_POS_TOPIC, 1);
  }

  void
  straight_line_robot_increment_and_publish(const double &increment_in_meters) {
    auto msg = std_msgs::Float32MultiArray();
    left_wheel_pos_ += SimpleRoboticsCppUtils::normalize_angle_2PI(
        increment_in_meters / wheel_radius_);
    right_wheel_pos_ = -left_wheel_pos_; // because right wheel's right hand
                                         // positive direction is opposite
    publish_odom({left_wheel_pos_, right_wheel_pos_});
  }

  void publish_odom(const std::pair<double, double> &wheel_pos_pair) {
    std_msgs::Float32MultiArray wheel_pos;
    wheel_pos.data.push_back(wheel_pos_pair.first);
    wheel_pos.data.push_back(wheel_pos_pair.second);
    odom_pub_.publish(wheel_pos);
    ros::Duration(0.1).sleep();
    ros::spinOnce();
  }

  Eigen::Matrix4d get_tf_matrix() const { return Eigen::Matrix4d(tf_matrix_); }
};

/**
 * @brief : we are testing protected functions using a wrapper class
 */
struct TestableDreamGMapper : public DreamGMapping::DreamGMapper {
private:
  ros::Publisher laser_pub_;

public:
  ~TestableDreamGMapper() { laser_pub_.shutdown(); }

  explicit TestableDreamGMapper(ros::NodeHandle nh) : DreamGMapper(nh) {
    particle_num_ = SMALL_PARTICLE_NUM;
    laser_pub_ = nh.advertise<sensor_msgs::LaserScan>("scan", 1);
  }
  void normalize_weights(std::vector<Particle> &particles) {
    DreamGMapping::DreamGMapper::normalize_weights(particles);
  }
  std::vector<unsigned int>
  get_resampled_indices(const std::vector<Particle> &particles) const {
    return DreamGMapping::DreamGMapper::get_resampled_indices(particles);
  }

  void publish_wall_laser_scan(const double &distance) {
    // publish a range scan message to itself
    auto wall_laser_scan = create_wall_laser_scan(distance);
    laser_pub_.publish(*wall_laser_scan);
    ros::Duration(0.1).sleep();
    ros::spinOnce();
  }

  void test_initialization() {
    EXPECT_EQ(particle_num_, SMALL_PARTICLE_NUM);
    EXPECT_GT(motion_covariances_(0, 0), 0);
    EXPECT_GT(motion_covariances_(1, 1), 0);
    EXPECT_LT(log_prob_beam_not_found_in_kernel_, 0);
    // map_size_ (in pixels) must be odd
    EXPECT_EQ(map_size_ % 2, 1);
    EXPECT_LT(map_.info.origin.position.x, 0);
    EXPECT_LT(map_.info.origin.position.y, 0);
    for (unsigned int i = 1; i < motion_set_.size(); ++i) {
      EXPECT_NE(motion_set_[i], Eigen::Matrix4d::Identity());
    }
    ROS_INFO("Testing Initialization Passed");
  }

  void test_laser_scan_before_odom_straightline(const double &init_dist) {
    // TODO: organize this code into the initializer
    publish_wall_laser_scan(init_dist);
    EXPECT_EQ(received_first_laser_scan_, false);
    ROS_INFO("Passed test_laser_scan_before_odom_straightline");
  }

  // One vanilla test, we give good odom and laser point data.
  void test_with_laser_scan_straight_line(const double &distance) {
    // TODO: make a wall
    publish_wall_laser_scan(distance);
    Eigen::Matrix4d last_odom_pose_groud_truth = Eigen::Matrix4d::Identity();
    last_odom_pose_groud_truth(0, 3) = INITIAL_WALL_DIST - distance;
    EXPECT_TRUE(last_odom_pose_.isApprox(last_odom_pose_groud_truth, 1e-3));
    // laser_scan should have finished now.
    EXPECT_EQ(received_first_laser_scan_, true);
    // TODO
    // check at the end, if the each particle's pose is close enough, and weight
    // for (const auto &p : particles_) {
    //   std::cout << "p.pose:" << *(p.pose_traj_.back()) << std::endl;
    // }
  }
};

/**
 * @brief This class is composed of unit tests and one integration test.
 *
 */
class DreamGMapperTests : public ::testing::Test {
protected:
  TestableDreamGMapper *dream_gmapper;
  TestableDreamOdometer *dream_odometer;
  /**
   *here ros::~NodeHandle() would be called, but because the node is not
    started by the node handle,
    The node is still running, and all subscribers, and publishers are
    still preserved. The only thing changed is
    the node's reference count is 0
   */
  void SetUp() override {
    ros::NodeHandle nh("~");
    nh.setParam("wheel_dist", WHEEL_DIST);
    nh.setParam("wheel_diameter", WHEEL_DIAMETER);
    nh.setParam("d_v_std_dev", D_V_STD_DEV);
    nh.setParam("d_theta_std_dev", D_THETA_STD_DEV);
    nh.setParam("resolution", RESOLUTION);
    nh.setParam("beam_noise_variance", BEAM_NOISE_VARIANCE);
    nh.setParam("half_beam_kernel_size", HALF_BEAM_KERNEL_SIZE);
    nh.setParam("map_size_in_meters", MAP_SIZE_IN_METERS);
    nh.setParam("translation_active_threshold", TRANSLATION_ACTIVE_THRESHOLD);
    nh.setParam("angular_active_threshold", ANGULAR_ACTIVE_THRESHOLD);
    nh.setParam("particle_num", SMALL_PARTICLE_NUM);
    nh.setParam("skip_invalid_beams", true);
    nh.setParam("wheel_pos_topic", WHEEL_POS_TOPIC);
    nh.setParam("scan_topic", SCAN_TOPIC);
    nh.setParam("map_topic", MAP_TOPIC);

    dream_gmapper = new TestableDreamGMapper(nh);
    dream_odometer = new TestableDreamOdometer(nh);
  }
  void TearDown() override {
    delete dream_gmapper;
    delete dream_odometer;
  }
};

/**
 * -----------------------------------------------------------------
 * Odom Test
 * -----------------------------------------------------------------
 */

TEST_F(DreamGMapperTests, TestableDreamOdometer) {
  // go straight by WHEEL_DIAMETER * pi /4
  std::pair<double, double> current_wheel_pos = {0.0, 0.0};
  Eigen::Matrix4d groud_truth = Eigen::Matrix4d::Identity();
  Eigen::Matrix4d odom_base_link;

  current_wheel_pos.first += M_PI / 2.0;
  current_wheel_pos.second += -M_PI / 2.0;
  dream_odometer->publish_odom(current_wheel_pos);
  odom_base_link = dream_odometer->get_tf_matrix();
  groud_truth(0, 3) = WHEEL_DIAMETER * M_PI / 4.0;
  EXPECT_TRUE(odom_base_link.isApprox(groud_truth, 1e-3));

  // turn right on the spot
  current_wheel_pos.first += M_PI / 2.0;
  current_wheel_pos.second += M_PI / 2.0;
  dream_odometer->publish_odom(current_wheel_pos);
  odom_base_link = dream_odometer->get_tf_matrix();
  double theta = -WHEEL_DIAMETER / WHEEL_DIST * M_PI / 2.0;
  Eigen::AngleAxisd rotationZ(theta, Eigen::Vector3d(0, 0, 1));
  groud_truth.block<3, 3>(0, 0) = rotationZ.toRotationMatrix();
  EXPECT_TRUE(odom_base_link.isApprox(groud_truth, 1e-3));

  // turn by 2pi in n segments with the left wheel being still. So the robot
  // will come back to its previous pose
  const double WHEEL_RADIUS = WHEEL_DIAMETER / 2.0;
  const int N_SEGMENT = std::ceil(WHEEL_DIST * 2 / WHEEL_RADIUS);
  const double right_wheel_delta =
      (2.0 * M_PI * WHEEL_DIST / N_SEGMENT) / WHEEL_RADIUS;
  std::cout << "N_SEGMENT: " << N_SEGMENT
            << ", right wheel delta: " << right_wheel_delta << std::endl;
  for (int i = 0; i < N_SEGMENT; ++i) {
    current_wheel_pos.second += right_wheel_delta;
    dream_odometer->publish_odom(current_wheel_pos);
  }
  odom_base_link = dream_odometer->get_tf_matrix();
  EXPECT_TRUE(odom_base_link.isApprox(groud_truth, 1e-3));
}

/**
 * -----------------------------------------------------------------
 * DreamGMapper Test
 * -----------------------------------------------------------------
 */
// Low TODO: faulty test
// TEST_F(DreamGMapperTests, TestParticleNormalize) {
//   std::vector<Particle> particles(LARGE_PARTICLE_NUM, Particle());
//   for (auto &p : particles) {
//     p.weight_ = 1.0;
//   }
//   dream_gmapper->normalize_weights(particles);
//   std::for_each(particles.begin(), particles.end(), [](Particle &p) {
//     EXPECT_NEAR(p.weight_, 1.0 / LARGE_PARTICLE_NUM, 1e-5);
//     ;
//   });

//   for (unsigned int i = 0; i < particles.size(); ++i) {
//     // This could be flaky. so there's 10 weights in total, and only [0, n *
//     // 100] can have over 10 counts
//     particles[i].weight_ = (i % 100 == 0) ? 100.0 : 1.0;
//   }
//   auto indices = dream_gmapper->get_resampled_indices(particles);
//   std::unordered_map<unsigned int, unsigned int> index_map;
//   for (const auto &i : indices) {
//     index_map[i]++;
//   }

//   unsigned int number_of_indices_over_10 = 0;
//   for (const auto &pair : index_map) {
//     if (pair.second > 10) {
//       number_of_indices_over_10++;
//       EXPECT_EQ(pair.first % 100, 0);
//     }
//   }
//   EXPECT_GT(number_of_indices_over_10, 10);
// }

// ================================================================================================
// Integration Tests. They are called inside one single GTest Function IN
// SEQUENCE One flaw in these tests is we do topic publication inside these
// functions - we do this for visual convenience technically, that's a "leaky
// abstraction"
// ================================================================================================

/**
 * @brief This is an integration test that mocks a simple scenario of the
 robot:
 * 1. The robot starts off 1m behind a wall. It first receives a laser scan
 * 2. The robot moves 0.1m forward, and receive a odom message
 * 3. The robot receives a scan message again
 * 4. The robot receives moves 0.1m forward again, and receive another odom
 * message
 * 5. The robot receives a scan message.
 */
TEST_F(DreamGMapperTests, IntegrationTest) {
  dream_gmapper->test_initialization();
  double distance = INITIAL_WALL_DIST;
  dream_gmapper->test_laser_scan_before_odom_straightline(distance);
  constexpr double forward_increment =
      1.1 *
      TRANSLATION_ACTIVE_THRESHOLD; // 0.2m
                                    // TODO: add tear down of the dream_odometer
                                    // , because its publisher is still alive
  // start from 0, so first initialize the DreamGMapper with odom and laser scan
  dream_odometer->straight_line_robot_increment_and_publish(0);
  dream_gmapper->test_with_laser_scan_straight_line(distance);
  for (int i = 0; i < 6; i++) {
    std::cout << "============================= odom pub and laser scan test "
                 "============================="
              << std::endl;
    distance -= forward_increment;
    dream_odometer->straight_line_robot_increment_and_publish(
        forward_increment);
    dream_gmapper->test_with_laser_scan_straight_line(distance);
  }
  // publish the map
  ros::Duration(5.0).sleep();
}

// Run all the tests that were declared with TEST()
int main(int argc, char **argv) {
  // Don't forget to call ros::init function as it's needed for ros
  // infrastructure
  ros::init(argc, argv, "test_dream_gmapping");
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
