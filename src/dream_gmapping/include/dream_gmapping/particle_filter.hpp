#pragma once
#include <functional>
#include <iostream>
#include <memory>
#include <unordered_map>
// for std::make_tuple
#include <tuple>
#include <vector>

// TODO: transfer to own library
namespace Rigid2D {
struct Pose {
  double x, y, theta;
  Pose(double x_, double y_, double theta_) : x(x_), y(y_), theta(theta_) {}
};

struct Pixel2DWithCount {
  int x, y;
  unsigned int hit_count_;
  unsigned int total_count_ = 1;
  Pixel2DWithCount(int x_, int y_, unsigned int hit_count_)
      : x(x_), y(y_), hit_count_(hit_count_) {}
  /**
   * @brief Return true if the point is considered occupied, otherwise false.
   This is based on the "reflection map". When added to the PointAccumulator,
   this point's count is at least 1 So, we just use a bool to represent
   occupancy
   */
  bool is_full() const {
    // We set 0.5 as known/unknown boundary. So, if
    return (hit_count_ << 1) > total_count_;
  }
};

inline std::ostream &operator<<(std::ostream &stream,
                                const Pixel2DWithCount &p) {
  stream << "(" << p.x << " " << p.y << ")";
  // return the object to allow chaining
  return stream;
}

inline bool operator==(const Pixel2DWithCount &p1, const Pixel2DWithCount &p2) {
  return p1.x == p2.x && p1.y == p2.y;
}

}; // namespace Rigid2D

namespace std {
// TODO: what does this anonymous template func do?
template <> struct hash<Rigid2D::Pixel2DWithCount> {
  std::hash<long> _hasher;
  size_t operator()(const Rigid2D::Pixel2DWithCount &p) const {
    // shifting 4 bytes
    constexpr size_t _shift_bits_num = sizeof(int) * 8;
    return _hasher(static_cast<long>(p.x) << _shift_bits_num |
                   static_cast<long>(p.y));
  }
};
} // namespace std

namespace DreamGMapping {

class PointAccumulator {
private:
  // { long: {<x,y>, count}} = 8 + 4 + 4 + 4 = 20 bytes. That translates to 20MB
  // of memory. In reality, 67MB for 100 Particles, 10000 points
  std::unordered_map<long, Rigid2D::Pixel2DWithCount> count_map_;

public:
  void add_point(Rigid2D::Pixel2DWithCount &&p) {
    count_map_.emplace(std::hash<Rigid2D::Pixel2DWithCount>()(p), std::move(p));
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
  std::vector<std::shared_ptr<Rigid2D::Pose>> pose_traj_;
  PointAccumulator laser_point_accumulation_map_;
  // all ctor and dtors are default
};

// create a particle
// copy the particle
// delete it, show memory usage. custom deleter?

}; // namespace DreamGMapping