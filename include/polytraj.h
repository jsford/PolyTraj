#ifndef POLYTRAJ_POLYTRAJ_H
#define POLYTRAJ_POLYTRAJ_H

#include <Eigen/Dense>

namespace PolyTraj {
namespace Path {

/// A PathState contains vehicle x, y, theta, and curvature values.
enum StateLayoutEnum : int {
  SX = 0, SY = 1, ST = 2, SK = 3, SSZ = 4
};
using State = Eigen::Vector4d;

/// A Path is a list of PathStates.
using Path = Eigen::Matrix<double, SSZ, Eigen::Dynamic>;

/// Generate a path from the initial to the final state..
Path generate(const State &initialState, const State &finalState,
              int points = 101);

} // namespace Path

namespace Trajectory {

/// A Trajectory::State contains vehicle x, y, theta, curvature, and velocity values.
enum StateLayoutEnum : int {
  SX = 0, SY = 1, ST = 2, SK = 3, SV = 4, SA = 5, SSZ = 6
};
using State = Eigen::Matrix<double, SSZ, 1>;

/// A Trajectory is a list of Trajectory::States
using Trajectory = Eigen::Matrix<double, SSZ, Eigen::Dynamic>;

/// Generate a trajectory from the initial to the final state.
Trajectory generate(const State &initialState, const State &finalState,
                    int kDotDeg, int vDotDeg, int points = 101);

} // namespace Trajectory
} // namespace PolyTraj
#endif  // POLYTRAJ_POLYTRAJ_H
