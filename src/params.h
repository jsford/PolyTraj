#ifndef POLYTRAJ_PARAMS_H
#define POLYTRAJ_PARAMS_H

#include <Eigen/Dense>
#include "params.h"
#include "polynomial.h"

namespace PolyTraj {
namespace Trajectory {
/// Parameterizes a control sequence for controlling the vehicle along a
/// trajectory.
struct Params {
  double T;
  Polynomial kDotPoly;
  Polynomial aDotPoly;

  Params(double T, const Polynomial &kDotPoly, const Polynomial &aDotPoly);

  explicit Params(const Eigen::VectorXd &params, int kDotDegree,
                  int vDotDegree);

  double operator()(double s) const;

  Eigen::VectorXd vector() const;
};

}  // namespace Trajectory

namespace Path {
/// Parameterizes a control sequence for controlling the vehicle along a path.
struct Params {
  double S;
  Polynomial kDotPoly;

  Params(double S, const Polynomial &kDotPoly);

  explicit Params(const Eigen::VectorXd &params);

  double operator()(double s) const;

  Eigen::VectorXd vector() const;
};

}  // namespace Path
}  // namespace PolyTraj
#endif  // POLYTRAJ_PARAMS_H
