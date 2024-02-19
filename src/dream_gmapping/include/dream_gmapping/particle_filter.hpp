#pragma once
#include "simple_robotics_cpp_utils/rigid2d.hpp"

namespace DreamGMapping {

class PointAccumulator {
private:
  // { long: {<x,y>, count}} = 8 + 4 + 4 + 4 = 20 bytes. That translates to 20MB
  // of memory. In reality, 67MB for 100 Particles, 10000 points
  std::unordered_map<long, SimpleRoboticsCppUtils::Pixel2DWithCount> count_map_;

public:
  void add_point(SimpleRoboticsCppUtils::Pixel2DWithCount &&p) {
    count_map_.emplace(std::hash<SimpleRoboticsCppUtils::Pixel2DWithCount>()(p), std::move(p));
  }
};

// motion model update -> pixel value -> transform laser points length /
// resolution * multiplier -> find in point accumulator for scan matching. ->
// T_trans to t_point -> adjust particle pose T_pose -> get T_pose_in_pixel ->
// add tp pointaccumulator laser points transform + T_pose_in_pixel
struct Particle {
  double _weight;
  // TODO: pose or pixel2D?: drawing from motion model needs it, then  and
  // refining
  std::vector<std::shared_ptr<SimpleRoboticsCppUtils::Pose2D>> pose_traj_;
  PointAccumulator laser_point_accumulation_map_;
  // all ctor and dtors are default
};

// create a particle
// copy the particle
// delete it, show memory usage. custom deleter?

}; // namespace DreamGMapping