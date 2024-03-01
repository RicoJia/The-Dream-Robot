#include "dream_gmapping/dream_gmapper.hpp"
#include "dream_gmapping/dream_gmapping_utils.hpp"
#include "ros/node_handle.h"
#include "sensor_msgs/LaserScan.h"
#include "simple_robotics_cpp_utils/performance_utils.hpp"
#include <cmath>
#include <gtest/gtest.h>
#include <simple_robotics_cpp_utils/rigid2d.hpp>
#include <unistd.h>

constexpr double WHEEL_DIST = 1;
using DreamGMapping::Particle;

/**
 * @brief : we are testing protected functions using a wrapper class
 *
 */
struct TestableDreamGMapper : public DreamGMapping::DreamGMapper {
public:
  explicit TestableDreamGMapper(ros::NodeHandle nh_) : DreamGMapper(nh_) {}
  void normalize_weights(std::vector<Particle> &particles) {
    DreamGMapping::DreamGMapper::normalize_weights(particles);
  }
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
    dream_gmapper = new TestableDreamGMapper(nh);
    ros::param::set("wheel_dist", WHEEL_DIST);
  }
  void TearDown() override { delete dream_gmapper; }
};

TEST_F(DreamGMapperTests, TestInitialization) {
  ROS_INFO("Testing Initialization");
  // Note: in the dream_gmapper module, we initialized a private node handle
  // with /dream_gmapping
  ros::NodeHandle nh("~");

  auto laser_pub = nh.advertise<sensor_msgs::LaserScan>("scan", 1);
  laser_pub.publish(sensor_msgs::LaserScan());
  // sleep in 10ms, which is necessary before spinOnce so subscriber could
  // the callback
  usleep(10000);
  ros::spinOnce();
}

TEST_F(DreamGMapperTests, TestParticleNormalize) {
  std::vector<Particle> particles(4, Particle());
  for (auto &p : particles) {
    p.weight_ = 1.0;
  }
  dream_gmapper->normalize_weights(particles);
  std::for_each(particles.begin(), particles.end(), [](Particle &p) {
    EXPECT_NEAR(p.weight_, 0.25, 1e-5);
    ;
  });
}

// Run all the tests that were declared with TEST()
int main(int argc, char **argv) {
  // Don't forget to call ros::init function as it's needed for ros
  // infrastructure
  ros::init(argc, argv, "test_dream_gmapping");
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
