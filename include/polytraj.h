#ifndef POLYTRAJ_POLYTRAJ_H
#define POLYTRAJ_POLYTRAJ_H

#include <Eigen/Dense>

namespace PolyTraj {

/// A PathState contains vehicle x, y, theta, and curvature values.
enum PathStateLayoutEnum : int {
  PSX = 0, PSY = 1, PST = 2, PSK = 3
};
using PathState = Eigen::Vector4d;

/// A Path is a list of PathStates.
using Path = Eigen::Matrix<double, 4, Eigen::Dynamic>;

/// Generate a path from the initial to the final state..
Path generatePath(const PathState &initialState, const PathState &finalState,
                  int points = 101);

/// A TrajState contains vehicle x, y, theta, curvature, and velocity values.
enum TrajStateLayoutEnum : int {
  TSX = 0, TSY = 1, TST = 2, TSK = 3, TSV = 4
};
using TrajState = Eigen::Matrix<double, Eigen::Dynamic, 1>;

/// A Traj is a list of TrajStates
using Traj = Eigen::Matrix<double, 5, Eigen::Dynamic>;

/// Generate a trajectory from the initial to the final state.
Traj generateTraj(const TrajState &initialState, const TrajState &finalState,
                  int points = 101);

}  // namespace PolyTraj
#endif  // POLYTRAJ_POLYTRAJ_H
