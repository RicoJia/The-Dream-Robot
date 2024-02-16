#include "dream_gmapping/dream_gmapper.hpp"
#include "ros/node_handle.h"
#include <gtest/gtest.h>
#include <unistd.h>


class DreamGMapperTests : public ::testing::Test {
protected:
  DreamGMapping::DreamGMapper *dream_gmapper;
  // overriding the default SetUp Function
  void SetUp() override {
    ros::NodeHandle nh("~");
    dream_gmapper = new DreamGMapping::DreamGMapper(nh);
    // here ros::~NodeHandle() would be called, but because the node is not started by the node handle,
    // The node is still running, and all subscribers, and publishers are still preserved. The only thing changed is
    // the node's reference count is 0
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
  // sleep in 10ms, which is necessary before spinOnce so subscriber could get
  // the callback
  usleep(10000);
  ros::spinOnce(); 
}

// Run all the tests that were declared with TEST()
int main(int argc, char **argv) {
  // Don't forget to call ros::init function as it's needed for ros
  // infrastructure
  ros::init(argc, argv, "test_dream_gmapping");
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
