#ifndef POLYTRAJ_PATH_H
#define POLYTRAJ_PATH_H

#include <params.h>
#include <polynomial.h>
#include <polytraj.h>

#include <cppoptlib/meta.h>
#include <cppoptlib/problem.h>
#include <Eigen/Dense>

namespace polytraj {
namespace path {

/// Define the vehicle dynamics xDot = f(s, x, u(s))
State dynamics(double s, const State &x, const Params &params);

/// Guess path parameters.
Params initParams(const State &xs, const State &xe, int kDotDeg);

/// Create a path! Go from start to end.
Params optimizeParams(const State &xs, const State &xe, int kDotDeg = 4);

class OptimizationProblem : public ::cppoptlib::Problem<double> {
public:
  State xs;
  State xe;
  int N;

  OptimizationProblem(const State &xs, const State &xe, int N = 100);

  double value(const cppoptlib::Problem<double>::TVector &q);
  // void gradient(const cppoptlib::Problem<double>::TVector& q,
  // cppoptlib::Problem<double>::TVector& grad);
  // void hessian(const cppoptlib::Problem<double>::TVector& q,
  // cppoptlib::Problem<double>::THessian& hess);
};

}  // namespace path
}  // namespace polytraj
#endif  // POLYTRAJ_PATH_H
