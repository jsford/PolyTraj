#ifndef POLYTRAJ_PARAMS_H
#define POLYTRAJ_PARAMS_H

#include <Eigen/Dense>
#include "params.h"
#include "polynomial.h"

namespace polytraj {
namespace trajectory {
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

}  // namespace trajectory

namespace path {
/// Parameterizes a control sequence for controlling the vehicle along a path.
struct Params {
  double S;
  Polynomial kDotPoly;

  Params(double S, const Polynomial &kDotPoly);

  explicit Params(const Eigen::VectorXd &params);

  double operator()(double s) const;

  Eigen::VectorXd vector() const;
};

}  // namespace path
}  // namespace polytraj
#endif  // POLYTRAJ_PARAMS_H
