#ifndef POLYTRAJ_PARAMS_H
#define POLYTRAJ_PARAMS_H

#include <Eigen/Dense>
#include "params.h"
#include "polynomial.h"

namespace PolyTraj {

/// Parameterizes a control sequence for controlling the vehicle along a
/// trajectory.
struct TrajParams {
  double S;
  Polynomial kDotPoly;
  Polynomial vDotPoly;

  TrajParams(double S, const Polynomial &kDotPoly, const Polynomial &vDotPoly);

  explicit TrajParams(const Eigen::VectorXd &params, int kDotDegree,
                      int vDotDegree);

  double operator()(const double s) const;

  Eigen::VectorXd vector() const;
};

/// Parameterizes a control sequence for controlling the vehicle along a path.
struct PathParams {
  double S;
  Polynomial kDotPoly;

  PathParams(double S, const Polynomial &kDotPoly);

  explicit PathParams(const Eigen::VectorXd &params);

  double operator()(const double s) const;

  Eigen::VectorXd vector() const;
};

}  // namespace PolyTraj
#endif  // POLYTRAJ_PARAMS_H
