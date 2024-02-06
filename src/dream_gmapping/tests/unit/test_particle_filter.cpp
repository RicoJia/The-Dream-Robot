#include "dream_gmapping/particle_filter.hpp"
#include <cstdint>
#include <gtest/gtest.h>
#include <iostream>
#include <memory>
#include <sys/resource.h>
#include <vector>

using namespace DreamGMapping;

// constants
// 40000 traj points
uint TRAJ_POINT_NUM = 10 * 10 / (5 * 5) * 1e4;
uint PARTICLE_NUM = 100;
uint TRIAL_NUM = 5;

long get_memory_usage() {
  struct rusage usage;
  getrusage(RUSAGE_SELF, &usage);
  return usage.ru_maxrss; // in kb
}

/**
 * @brief Best case for particle creation is when all particles share the same
 * underlying poses.
 *
 * @param p_ptr : pointer to the particle to copy off of.
 * @return std::vector<std::unique_ptr<Particle>> : vector of the particles
 */
void get_list_of_particle_best_case(
    const std::unique_ptr<Particle> &p_ptr,
    std::vector<std::unique_ptr<Particle>> &p_list) {
  p_list.clear();
  p_list.reserve(PARTICLE_NUM);
  for (int i = 0; i < PARTICLE_NUM; i++) {
    p_list.push_back(std::make_unique<Particle>(*p_ptr));
  }
  // c++ 11 already uses move semantics to return local objects
}

// Naming conventions are important here
TEST(ParticleFilterTests, TestParticleMemoryBestCase) {
  std::vector<long> mem_usages;
  // create a unique_ptr as we normally would in production
  // Note: this helps us share the same tnode.
  auto p_ptr = std::make_unique<Particle>();
  p_ptr->_weight = 1;
  for (int i = 0; i < TRAJ_POINT_NUM; i++) {
    p_ptr->_pose_traj.push_back(std::make_shared<Rigid2D::Pose>(i, i, i));
  }
  std::vector<std::unique_ptr<Particle>> p_list;
  for (int i = 0; i < TRIAL_NUM; i++) {
    get_list_of_particle_best_case(p_ptr, p_list);
    long mem_usage = get_memory_usage();
    std::cout << "memory usage " << mem_usage << std::endl;
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
    p_list.back()->_weight = 1;
    for (int i = 0; i < TRAJ_POINT_NUM; i++) {
      p_list.back()->_pose_traj.push_back(
          std::make_shared<Rigid2D::Pose>(i, i, i));
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
    std::cout << "memory usage " << mem_usage << std::endl;
    mem_usages.push_back(mem_usage);
  }
  // `p_list.reserve()` also makes sure no extra memory is allocated while
  // vector is appended to. So we can have a higher likelihood that the program
  // reuses the previosly reserved memory, which will cap the memory usage of
  // the program. However, since we are creating new Pose objects, cpp could
  // still use new memory. So, we don't write a test for it as it could be
  // flaky.
}

// Run all the tests that were declared with TEST()
int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
