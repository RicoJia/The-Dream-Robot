#include "shared_test_utils.hpp"
#include <cstdint>
#include <gtest/gtest.h>
#include <iostream>
#include <memory>
#include <unordered_map>
#include <vector>

using namespace DreamGMapping;

// constants
// 40000 traj points
uint TRAJ_POINT_NUM = 10 * 10 / (5 * 5) * 1e4;
uint TRIAL_NUM = 5;
unsigned int OBSTACLE_COUNT = 20 * 1000;

/**
 * @brief Best case for particle creation is when all particles share the same
 * underlying poses.
 *
 * @param p_ptr : pointer to the particle to copy off of.
 * @return std::vector<std::unique_ptr<Particle>> : vector of the particles
 */
void get_list_of_particle_best_case(
    // This is could be resampling
    const std::unique_ptr<Particle> &p_ptr,
    std::vector<std::unique_ptr<Particle>> &p_list) {
  p_list.clear();
  p_list.reserve(PARTICLE_NUM);
  for (int i = 0; i < PARTICLE_NUM; i++) {
    p_list.emplace_back(std::make_unique<Particle>(*p_ptr));
  }
  // c++ 11 already uses move semantics to return local objects
}

// Naming conventions are important here
TEST(ParticleFilterTests, TestParticleMemoryBestCase) {
  std::vector<long> mem_usages;
  // create a unique_ptr as we normally would in production
  // Note: this helps us share the same tnode.
  auto p_ptr = std::make_unique<Particle>();
  p_ptr->weight_ = 1;
  for (int i = 0; i < TRAJ_POINT_NUM; i++) {
    p_ptr->pose_traj_.push_back(
        std::make_shared<SimpleRoboticsCppUtils::Pose2D>(i, i, i));
  }
  std::vector<std::unique_ptr<Particle>> p_list;
  for (int i = 0; i < TRIAL_NUM; i++) {
    get_list_of_particle_best_case(p_ptr, p_list);
    long mem_usage = get_memory_usage();
    std::cout << " Best case iteration" << i << " memory usage: " << mem_usage
              << std::endl;
    mem_usages.push_back(mem_usage);
  }
  for (int i = 1; i < TRIAL_NUM; i++) {
    ASSERT_EQ(mem_usages[i], mem_usages[i - 1])
        << "Memory Usage increases in best case particle creation! "
        << "Note we use ru_maxrss which measures the cpp memory usage. That "
           "may not instanenousely decrease,"
        << "but still we should check why memory increases";
  }
}

/**
 * @brief The worst case in particle creation is when particles are not shared,
 * and have to be created from scratch.
 * @param p_list : reference to vector.
 */
void get_list_of_particle_worst_case(
    std::vector<std::unique_ptr<Particle>> &p_list) {
  // here we must clear the vector. Otherwise we will be appending to the vector
  p_list.clear();
  p_list.reserve(PARTICLE_NUM);
  for (int i = 0; i < PARTICLE_NUM; i++) {
    p_list.push_back(std::make_unique<Particle>());
    p_list.back()->weight_ = 1;
    for (int i = 0; i < TRAJ_POINT_NUM; i++) {
      p_list.back()->pose_traj_.push_back(
          std::make_shared<SimpleRoboticsCppUtils::Pose2D>(i, i, i));
    }
  }
  // c++ 11 already uses move semantics to return local objects
}
// Naming conventions must be CamelCase. It's important here
TEST(ParticleFilterTests, TestParticleMemoryWorstCase) {
  std::vector<long> mem_usages;
  std::vector<std::unique_ptr<Particle>> p_list;
  for (int i = 0; i < TRIAL_NUM; i++) {
    // TODO: this is still failing
    get_list_of_particle_worst_case(p_list);
    long mem_usage = get_memory_usage();
    std::cout << " Worst case iteration" << i << " memory usage: " << mem_usage
              << std::endl;
    mem_usages.push_back(mem_usage);
  }
  // TODO: `p_list.reserve()` also makes sure no extra memory is allocated while
  // vector is appended to. So we can have a higher likelihood that the program
  // reuses the previosly reserved memory, which will cap the memory usage of
  // the program. However, since we are creating new Pose objects, cpp could
  // still use new memory. So, we don't write a test for it as it could be
  // flaky.
}

/**********************************Utils Tests*********************************/

TEST(DreamGMapperUtilsTests, TestPointCloudUtils) {
  boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> prev_cloud(
      new pcl::PointCloud<pcl::PointXYZ>());
  long init_memory = get_memory_usage();
  {
    for (int i = 0; i < 1000; i++) {
      boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> next_cloud(
          new pcl::PointCloud<pcl::PointXYZ>());
      bool filled_success = DreamGMapping::fill_point_cloud(
          create_wall_laser_scan(1), prev_cloud);

      assert(filled_success && "Point Cloud Filling Failure");
      filled_success = DreamGMapping::fill_point_cloud(
          create_wall_laser_scan(8), next_cloud);
      assert(filled_success && "Point Cloud Filling Failure");

      Eigen::Matrix4d T_icp_output = Eigen::Matrix4d::Identity();
      auto T_init_guess =
          SimpleRoboticsCppUtils::screw_displacement_2d_to_body_frame_transform(
              std::pair<double, double>(0, 0));

      bool converge = DreamGMapping::icp_2d(prev_cloud, next_cloud,
                                            T_init_guess, T_icp_output);
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
  bool filled_success = DreamGMapping::fill_point_cloud(scan_msg, cloud);
  assert(filled_success && "point cloud filling failure");
  auto rotated_cloud =
      DreamGMapping::get_point_cloud_in_world_frame(pose, cloud);
  // This is tricky - origin is (-1, 1) in the body frame of the body frame
  SimpleRoboticsCppUtils::Pose2D pose_back_to_origin{-1, 1, -M_PI / 2};
  auto new_rotated_cloud = DreamGMapping::get_point_cloud_in_world_frame(
      pose_back_to_origin, rotated_cloud);
  EXPECT_EQ(cloud->width, new_rotated_cloud->width);
  for (unsigned int i = 0; i < new_rotated_cloud->width; i++) {
    EXPECT_NEAR(cloud->points[i].x, new_rotated_cloud->points[i].x, 1e-4);
    EXPECT_NEAR(cloud->points[i].y, new_rotated_cloud->points[i].y, 1e-4);
  }
}

TEST(DreamGMapperUtilsTests, PixelizePointCloudTest) {
  PclCloudPtr cloud(new pcl::PointCloud<pcl::PointXYZ>);
  const double RESOLUTION = 1;
  cloud->resize(1);
  cloud->points[0].x = RESOLUTION * 0.9;
  cloud->points[0].y = RESOLUTION * 0.9;
  DreamGMapping::pixelize_point_cloud(cloud, RESOLUTION);
  EXPECT_EQ(cloud->points[0].x, 0);
  EXPECT_EQ(cloud->points[0].y, 0);
}

TEST(DreamGMapperUtilsTests, PointAccumulatorTest) {
  DreamGMapping::PointAccumulator pa;
  pa.add_point(0, 0, false);
  pa.add_point(0, 0, false);
  pa.add_point(0, 0, true);
  auto [hit_count, total_count] = pa.get_counts(0, 0);
  EXPECT_EQ(pa.is_full(SimpleRoboticsCppUtils::Pixel2DWithCount(0, 0)), false);
  EXPECT_EQ(hit_count, 1);
  EXPECT_EQ(total_count, 3);
  std::cout << "hit_count: " << hit_count << " total_count: " << total_count
            << std::endl;

  auto [hit_counts2, total_counts2] = pa.get_counts(1, 0);
  EXPECT_EQ(hit_counts2, 0);
  EXPECT_EQ(total_counts2, 0);

  pa.add_point(1, 0, true);
  pa.add_point(1, 1, true);
  pa.add_point(1, -1, false);
  pa.add_point(-2, -2, true);
  std::vector<int8_t> data;
  // map_size has been made sure to be odd
  // so [-2, 1] along x and y directions
  unsigned int map_size = 5, origin_offset = map_size * map_size / 2;
  pa.fill_ros_map(data, map_size, origin_offset);
  EXPECT_EQ(data.at(0), 100);  // (-2,-2)
  EXPECT_EQ(data.at(12), 0);   // (0,0)
  EXPECT_EQ(data.at(8), 0);    // (1,-1)
  EXPECT_EQ(data.at(18), 100); // (1,1)
}

TEST(DreamGMapperUtilsTests, FindMostWeightedParticleIndex) {
  const unsigned int num_particles = 5;
  auto particles = std::vector<Particle>(num_particles, Particle());
  for (auto &p : particles) {
    p.weight_ = 1.0 / 5.0;
  }
  particles[num_particles - 1].weight_ = 1.0;
  unsigned int best_particle_index = 0;
  find_most_weighted_particle_index(particles, best_particle_index);
  EXPECT_EQ(best_particle_index, num_particles - 1);

  std::unordered_map<unsigned int, unsigned int> counts{
      {40, 2},
      {12, 1},
      {14, 5},
      {11, 7},
  };
  find_most_weighted_particle_index(counts, best_particle_index);
  EXPECT_EQ(best_particle_index, 11);
}

// Run all the tests that were declared with TEST()
int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
