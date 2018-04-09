#ifndef POLYTRAJ_PATH_H
#define POLYTRAJ_PATH_H

#include <params.h>
#include <polynomial.h>
#include <polytraj.h>

#include <cppoptlib/meta.h>
#include <cppoptlib/problem.h>
#include <Eigen/Dense>

namespace PolyTraj {

/// Define the vehicle dynamics xDot = f(s, x, u(s))
PathState dynamics(double s, const PathState &x, const PathParams &params);

/// Guess path parameters.
PathParams initPathParams(const PathState &xs, const PathState &xe);

/// Create a path! Go from start to end.
PathParams optimizePath(const PathState &xs, const PathState &xe);

class PathOptimizationProblem : public ::cppoptlib::Problem<double> {
 public:

  PathState xs;
  PathState xe;
  int N;

  PathOptimizationProblem(const PathState &xs, const PathState &xe,
                          int N = 100);

  double value(const cppoptlib::Problem<double>::TVector &q);
  // void gradient(const cppoptlib::Problem<double>::TVector& q,
  // cppoptlib::Problem<double>::TVector& grad);
  // void hessian(const cppoptlib::Problem<double>::TVector& q,
  // cppoptlib::Problem<double>::THessian& hess);
};

}  // namespace PolyTraj
#endif  // POLYTRAJ_PATH_H
