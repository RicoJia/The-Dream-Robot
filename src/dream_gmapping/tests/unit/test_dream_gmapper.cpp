#include "dream_gmapping/dream_gmapper.hpp"
#include "ros/node_handle.h"
#include "sensor_msgs/LaserScan.h"
#include "simple_robotics_cpp_utils/performance_utils.hpp"
#include <cmath>
#include <gtest/gtest.h>
#include <simple_robotics_cpp_utils/rigid2d.hpp>
#include <unistd.h>

constexpr double WHEEL_DIST = 1;

// Creating a "wall" along the y axis at x=distance
// The laser scan->frame on laser scan->with a. specified distance
// Note: the other half of the scan is max_distance
sensor_msgs::LaserScan::ConstPtr
create_wall_laser_scan(const double &distance) {
  boost::shared_ptr<sensor_msgs::LaserScan> scan(new sensor_msgs::LaserScan());
  constexpr int NUM_POINTS = 360;
  scan->ranges = std::vector<float>();
  scan->header.frame_id = "laser";
  scan->angle_min = 0;
  scan->angle_max = 2 * M_PI;
  // Positive is counter clockwise
  scan->angle_increment = 2 * M_PI / NUM_POINTS;
  scan->range_min = 0.0;
  scan->range_max = 100;
  double angle = scan->range_min;
  for (unsigned int i = 0; i < NUM_POINTS; i++) {
    if (std::abs(std::cos(angle)) < std::numeric_limits<float>::epsilon()) {
      scan->ranges[i] = scan->range_max;
    } else if (M_PI / 2.0 <= angle && angle <= M_PI * 3.0 / 2.0) {
      scan->ranges.push_back(scan->range_max);
    } else {
      scan->ranges.push_back(distance / std::cos(angle));
    }
    angle += scan->angle_increment;
  }
  return sensor_msgs::LaserScan::ConstPtr(scan);
}

TEST(DreamGMapperUtilsTests, TestPointCloudUtils) {
  boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> prev_cloud(
      new pcl::PointCloud<pcl::PointXYZ>());
  long init_memory = get_memory_usage();
  {
    for (int i = 0; i < 1000; i++) {
      boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> next_cloud(
          new pcl::PointCloud<pcl::PointXYZ>());
      bool filled_success =
          RosUtils::fill_point_cloud(create_wall_laser_scan(1), prev_cloud);

      assert(filled_success && "Point Cloud Filling Failure");
      filled_success =
          RosUtils::fill_point_cloud(create_wall_laser_scan(8), next_cloud);
      assert(filled_success && "Point Cloud Filling Failure");

      Eigen::Matrix4d T_icp_output = Eigen::Matrix4d::Identity();
      bool converge =
          RosUtils::icp_2d(prev_cloud, next_cloud,
                           std::pair<double, double>(0, 0), T_icp_output);
      prev_cloud = next_cloud;
    }
  }
  long after_memory = get_memory_usage();
  std::cout << "memory usage: " << (after_memory - init_memory) << ""
            << std::endl;
}

TEST(DreamGMapperUtilsTests, PointCloudInWorldFrameTest) {
  SimpleRoboticsCppUtils::Pose2D pose{1, 1, M_PI / 2};
  auto scan_msg = create_wall_laser_scan(1);
  boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> cloud(
      new pcl::PointCloud<pcl::PointXYZ>());
  bool filled_success = RosUtils::fill_point_cloud(scan_msg, cloud);
  assert(filled_success && "point cloud filling failure");
  auto rotated_cloud = RosUtils::get_point_cloud_in_world_frame(pose, cloud);
  // This is tricky - origin is (-1, 1) in the body frame of the body frame
  SimpleRoboticsCppUtils::Pose2D pose_back_to_origin{-1, 1, -M_PI / 2};
  auto new_rotated_cloud = RosUtils::get_point_cloud_in_world_frame(
      pose_back_to_origin, rotated_cloud);
  EXPECT_EQ(cloud->width, new_rotated_cloud->width);
  for (unsigned int i = 0; i < new_rotated_cloud->width; i++) {
    EXPECT_NEAR(cloud->points[i].x, new_rotated_cloud->points[i].x, 1e-4);
    EXPECT_NEAR(cloud->points[i].y, new_rotated_cloud->points[i].y, 1e-4);
  }
}

class DreamGMapperTests : public ::testing::Test {
protected:
  DreamGMapping::DreamGMapper *dream_gmapper;
  /**
   *here ros::~NodeHandle() would be called, but because the node is not
    started by the node handle,
    The node is still running, and all subscribers, and publishers are
    still preserved. The only thing changed is
    the node's reference count is 0
   */
  void SetUp() override {
    ros::NodeHandle nh("~");
    dream_gmapper = new DreamGMapping::DreamGMapper(nh);
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

// Run all the tests that were declared with TEST()
int main(int argc, char **argv) {
  // Don't forget to call ros::init function as it's needed for ros
  // infrastructure
  ros::init(argc, argv, "test_dream_gmapping");
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
