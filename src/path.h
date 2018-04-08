#ifndef POLYTRAJ_PATH_H
#define POLYTRAJ_PATH_H

#include <polytraj.h>
#include <polynomial.h>
#include <Eigen/Dense>

namespace PolyTraj {

/// Parameterizes a control sequence for controlling the vehicle along a path.
struct PathParams {
  double S;
  Polynomial kDotPoly;

  /// Construct PathParams from arclength and a control polynomial.
  PathParams(double S, const Polynomial &kDotPoly) : S(S), kDotPoly(kDotPoly) {}

  /// Construct PathParams from an Eigen::Vector.
  explicit PathParams(const Eigen::VectorXd &params)
    : S(params[0]), kDotPoly(params.tail(params.size() - 1)) {}

  /// Query the path params to find the controls as a function of arclength.
  double operator()(const double s) const { return kDotPoly(s); }

  /// Convert to Eigen::VectorXd.
  Eigen::VectorXd vector() {
    Eigen::VectorXd vec(1 + kDotPoly.deg());
    vec << S, kDotPoly.coeffs;
    return vec;
  }
};

/// Define the vehicle dynamics xDot = f(s, x, u(s))
PathState dynamics(double s, const PathState &x,
                   const PathParams &params);

/// Integrate the vehicle dynamics and return the path that resulted.
Path shootTrapezoidal(const PathState &xs, const PathParams &params, int N);

/// Integrate the vehicle dynamics and return the path that resulted.
Path shootSimpson(const PathState &xs, const PathParams &params, int N);

/// Guess path parameters.
PathParams initPathParams(const PathState &xs, const PathState &xe);

/// Create a path! Go from start to end.
PathParams optimizePath(const PathState &xs, const PathState &xe);

}  // namespace PolyTraj
#endif  // POLYTRAJ_PATH_H
