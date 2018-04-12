#ifndef POLYTRAJ_POLYTRAJ_H
#define POLYTRAJ_POLYTRAJ_H

#include <Eigen/Dense>

namespace polytraj {
namespace path {

/// A PathState contains vehicle x, y, theta, and curvature values.
enum StateLayoutEnum : int {
  SX = 0,   ///< Index of the x coordinate.
  SY = 1,   ///< Index of the y coordinate.
  ST = 2,   ///< Index of the theta coordinate.
  SK = 3,   ///< Index of the curvature coordinate.
  SSZ = 4   ///< Size of the state vector.
};
using State = Eigen::Matrix<double, SSZ, 1>;

/// A Path is a list of PathStates.
using Path = Eigen::Matrix<double, SSZ, Eigen::Dynamic>;

/// Generate a path from the initial to the final state..
Path generate(const State &initialState, const State &finalState,
              int points = 101);

} // namespace path

namespace trajectory {

/// A Trajectory::State contains vehicle x, y, theta, curvature, and velocity values.
enum StateLayoutEnum : int {
  SX = 0,   ///< Index of the x coordinate.
  SY = 1,   ///< Index of the y coordinate.
  ST = 2,   ///< Index of the theta coordinate.
  SK = 3,   ///< Index of the curvature coordinate.
  SV = 4,   ///< Index of the velocity coordinate.
  SA = 5,   ///< Index of the acceleration coordinate.
  SSZ = 6   ///< Size of the state vector.
};
using State = Eigen::Matrix<double, SSZ, 1>;

/// A Trajectory is a list of Trajectory::States
using Trajectory = Eigen::Matrix<double, SSZ, Eigen::Dynamic>;

/// Generate a trajectory from the initial to the final state.
Trajectory generate(const State &initialState, const State &finalState,
                    int kDotDeg, int vDotDeg, int points = 101);

} // namespace trajectory
} // namespace polytraj
#endif  // POLYTRAJ_POLYTRAJ_H
