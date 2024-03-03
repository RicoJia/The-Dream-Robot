#include "dream_gmapping/dream_gmapper.hpp"
#include "dream_gmapping/dream_gmapping_utils.hpp"
#include "ros/node_handle.h"
#include "sensor_msgs/LaserScan.h"
#include "simple_robotics_cpp_utils/performance_utils.hpp"
#include <cmath>
#include <gtest/gtest.h>
#include <simple_robotics_cpp_utils/rigid2d.hpp>
#include <unistd.h>
#include <unordered_map>

constexpr double WHEEL_DIST = 1;
using DreamGMapping::Particle;
const int PARTICLE_NUM = 1000;

/**
 * @brief : we are testing protected functions using a wrapper class
 *
 */
struct TestableDreamGMapper : public DreamGMapping::DreamGMapper {
public:
  explicit TestableDreamGMapper(ros::NodeHandle nh_) : DreamGMapper(nh_) {
    particle_num_ = PARTICLE_NUM;
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
    ROS_INFO("Testing Initialization");
    // Note: in the dream_gmapper module, we initialized a private node handle
    // with /dream_gmapping
    // TODO: check if laser scan has been published
  }

  void test_laser_before_odom() {
    // TODO: organize this code into the initializer
    ros::NodeHandle nh("~");
    auto laser_pub = nh.advertise<sensor_msgs::LaserScan>("scan", 1);
    laser_pub.publish(sensor_msgs::LaserScan());
    // sleep in 10ms, which is necessary before spinOnce so subscriber could
    // the callback
    usleep(10000);
    ros::spinOnce();
    // TODO: laser scan should have NOT been initialized
  }

  void test_odom() {}

  // One ginormous test
  void test_laser_scan() {}
};

class DreamGMapperTests : public ::testing::Test {
protected:
  // TODO
  //   DreamGMapping::DreamGMapper *dream_gmapper;
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
    ros::param::set("wheel_dist", WHEEL_DIST);
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

TEST_F(DreamGMapperTests, IntegrationTest) {
  dream_gmapper->test_initialization();
  dream_gmapper->test_laser_before_odom();
  dream_gmapper->test_odom();
  dream_gmapper->test_laser_scan();
}

// Run all the tests that were declared with TEST()
int main(int argc, char **argv) {
  // Don't forget to call ros::init function as it's needed for ros
  // infrastructure
  ros::init(argc, argv, "test_dream_gmapping");
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
