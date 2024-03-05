#include "dream_gmapping/dream_gmapper.hpp"
#include "dream_gmapping/dream_gmapping_utils.hpp"
#include "ros/node_handle.h"
#include "sensor_msgs/LaserScan.h"
#include "shared_test_utils.hpp"
#include "simple_robotics_cpp_utils/performance_utils.hpp"
#include "std_msgs/Float32MultiArray.h"
#include <cmath>
#include <gtest/gtest.h>
#include <simple_robotics_cpp_utils/rigid2d.hpp>
#include <unistd.h>
#include <unordered_map>

using DreamGMapping::Particle;

/**
 * @brief : we are testing protected functions using a wrapper class
 *
 */
struct TestableDreamGMapper : public DreamGMapping::DreamGMapper {
private:
  ros::Publisher laser_pub_;
  ros::Publisher odom_pub_;

public:
  explicit TestableDreamGMapper(ros::NodeHandle nh) : DreamGMapper(nh) {
    particle_num_ = PARTICLE_NUM;
    laser_pub_ = nh.advertise<sensor_msgs::LaserScan>("scan", 1);
    odom_pub_ = nh.advertise<std_msgs::Float32MultiArray>("odom", 1);
  }
  void normalize_weights(std::vector<Particle> &particles) {
    DreamGMapping::DreamGMapper::normalize_weights(particles);
  }
  std::vector<unsigned int>
  get_resampled_indices(const std::vector<Particle> &particles) const {
    return DreamGMapping::DreamGMapper::get_resampled_indices(particles);
  }

  // ================================================================================================
  // Integration Tests. They are called inside one single GTest Function IN
  // SEQUENCE One flaw in these tests is we do topic publication inside these
  // functions - we do this for visual convenience technically, that's a "leaky
  // abstraction"
  // ================================================================================================
  void test_initialization() {
    EXPECT_EQ(particle_num_, PARTICLE_NUM);
    EXPECT_EQ(wheel_dist_, WHEEL_DIST);
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

  void test_laser_before_odom() {
    // TODO: organize this code into the initializer
    ros::NodeHandle nh("~");
    laser_pub_.publish(sensor_msgs::LaserScan());
    // sleep in 10ms, which is necessary before spinOnce so subscriber could
    // the callback
    usleep(10000);
    ros::spinOnce();
    // TODO: laser scan should have NOT been initialized
    EXPECT_EQ(received_first_laser_scan_, false);
    ROS_INFO("Passed test_laser_before_odom");
  }

  void test_move_and_publish_odom(const double &forward_increment) {
    auto msg = std_msgs::Float32MultiArray();
    msg.data.clear();
    msg.data.push_back(0.1);
    msg.data.push_back(0.1);
    odom_pub_.publish(msg);
    // TODO: examine wheel odom
  }

  // TODO: angle wrap the delta, store the current wheel odom

  // One ginormous test
  void test_laser_scan(const double &distance) {}
};

/**
 * @brief This class is composed of unit tests and one integration test.
 *
 */
class DreamGMapperTests : public ::testing::Test {
protected:
  TestableDreamGMapper *dream_gmapper;
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
    nh.setParam("d_v_std_dev", D_V_STD_DEV);
    nh.setParam("d_theta_std_dev", D_THETA_STD_DEV);
    nh.setParam("resolution", RESOLUTION);
    nh.setParam("beam_noise_sigma_squared", BEAM_NOISE_SIGMA_SQUARED);
    nh.setParam("beam_kernel_size", BEAM_KERNEL_SIZE);
    nh.setParam("map_size_in_meters", MAP_SIZE_IN_METERS);
    dream_gmapper = new TestableDreamGMapper(nh);
  }
  void TearDown() override { delete dream_gmapper; }
};

TEST_F(DreamGMapperTests, TestParticleNormalize) {
  std::vector<Particle> particles(PARTICLE_NUM, Particle());
  for (auto &p : particles) {
    p.weight_ = 1.0;
  }
  dream_gmapper->normalize_weights(particles);
  std::for_each(particles.begin(), particles.end(), [](Particle &p) {
    EXPECT_NEAR(p.weight_, 1.0 / PARTICLE_NUM, 1e-5);
    ;
  });

  for (unsigned int i = 0; i < particles.size(); ++i) {
    // This could be flaky
    // so there's 10 weights in total, and only [0, n * 100] can have over 10
    // counts
    particles[i].weight_ = (i % 100 == 0) ? 100.0 : 1.0;
  }
  auto indices = dream_gmapper->get_resampled_indices(particles);
  std::unordered_map<unsigned int, unsigned int> index_map;
  for (const auto &i : indices) {
    // if (index_map.find(i) == index_map.end())
    //     index_map[i] = 1;
    // else
    index_map[i]++;
  }

  unsigned int number_of_indices_over_10 = 0;
  for (const auto &pair : index_map) {
    if (pair.second > 10) {
      number_of_indices_over_10++;
      EXPECT_EQ(pair.first % 100, 0);
      std::cout << "Indices with counts over 10: " << pair.first
                << "count: " << pair.second << std::endl;
    }
  }
  EXPECT_EQ(number_of_indices_over_10, 10);
}

/**
 * @brief This is an integration test that mocks a simple scenario of the robot:
 * 1. The robot starts off 1m behind a wall. It first receives a laser scan
 * 2. The robot moves 0.1m forward, and receive a odom message
 * 3. The robot receives a scan message again
 * 4. The robot receives moves 0.1m forward again, and receive another odom
 * message
 * 5. The robot receives a scan message.
 */
TEST_F(DreamGMapperTests, IntegrationTest) {
  dream_gmapper->test_initialization();
  dream_gmapper->test_laser_before_odom();
  double distance = 1.0;
  constexpr double forward_increment = 0.1;
  for (int i = 0; i < 2; i++) {
    dream_gmapper->test_move_and_publish_odom(forward_increment);
    distance -= forward_increment;
    dream_gmapper->test_laser_scan(distance);
  }
}

// Run all the tests that were declared with TEST()
int main(int argc, char **argv) {
  // Don't forget to call ros::init function as it's needed for ros
  // infrastructure
  ros::init(argc, argv, "test_dream_gmapping");
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
