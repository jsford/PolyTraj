#ifndef POLYTRAJ_TRAJECTORY_H
#define POLYTRAJ_TRAJECTORY_H

#include <params.h>
#include <polytraj.h>

#include <cppoptlib/meta.h>
#include <cppoptlib/problem.h>

namespace PolyTraj {
namespace Trajectory {

State dynamics(double t, const State &x, const Params &params);

Params initParams(const State &xs, const State &xe, int kDotDeg, int aDotDeg);

Params optimizeParams(const State &xs, const State &xe, int kDotDeg,
                      int aDotDeg);

class OptimizationProblem : public ::cppoptlib::Problem<double> {
public:
  State xs;
  State xe;
  int kDotDeg;
  int aDotDeg;
  int N;

  OptimizationProblem(const State &xs, const State &xe, int kDotDeg,
                      int aDotDeg, int N = 100);

  double value(const cppoptlib::Problem<double>::TVector &q);
  // void gradient(const cppoptlib::Problem<double>::TVector& q,
  // cppoptlib::Problem<double>::TVector& grad);
  // void hessian(const cppoptlib::Problem<double>::TVector& q,
  // cppoptlib::Problem<double>::THessian& hess);
};

}  // namespace Trajectory
}  // namespace PolyTraj
#endif  // POLYTRAJ_TRAJECTORY_H
