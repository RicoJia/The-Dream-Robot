#pragma once
#include <memory>
#include <vector>

// TODO: transfer to own library
namespace Rigid2D {
struct Pose {
  double x, y, theta;
  Pose(double x_, double y_, double theta_) : x(x_), y(y_), theta(theta_) {}
};

}; // namespace Rigid2D
namespace DreamGMapping {

struct Tnode {
  Rigid2D::Pose pose;
};
struct Particle {
  double _weight;
  std::vector<std::shared_ptr<Rigid2D::Pose>> _pose_traj;
  // all ctor and dtors are default
};

// create a particle
// copy the particle
// delete it, show memory usage. custom deleter?

}; // namespace DreamGMapping