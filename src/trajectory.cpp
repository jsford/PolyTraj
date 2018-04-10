#include "trajectory.h"
#include "path.h"
#include "shoot.h"

#include <cppoptlib/solver/neldermeadsolver.h>

namespace PolyTraj {
namespace Trajectory {

State dynamics(double t, const State &x, const Params &params) {
  State xDot;
  double v = x[SV];
  xDot << std::cos(x[ST]) * v,
    std::sin(x[ST]) * v,
    x[SK] * v,
    params.kDotPoly(t),
    x[SA],
    params.aDotPoly(t);
  return xDot;
}

Params initParams(const State &xs, const State &xe, int kDotDeg, int aDotDeg) {
  Path::Params pathParams =
    Path::optimizeParams(xs.head(Path::SSZ), xe.head(Path::SSZ), kDotDeg);

  Polynomial aDotPoly(Eigen::VectorXd::Zero(aDotDeg));

  return Params(pathParams.S, pathParams.kDotPoly, aDotPoly);
}

Params optimizeParams(const State &xs, const State &xe, int kDotDeg,
                      int aDotDeg) {
  Params params = initParams(xs, xe, kDotDeg, aDotDeg);

  cppoptlib::NelderMeadSolver<OptimizationProblem> solver;
  OptimizationProblem f(xs, xe, kDotDeg, aDotDeg, 100);

  Eigen::VectorXd p = params.vector();
  solver.minimize(f, p);
  return Params(p, kDotDeg, aDotDeg);
}

Trajectory generate(const State &initialState, const State &finalState,
                    int kDotDeg, int aDotDeg, int points) {
  Params params = optimizeParams(initialState, finalState, kDotDeg, aDotDeg);

  return shootSimpson(dynamics, initialState, params.T, points - 1, params);
}

OptimizationProblem::OptimizationProblem(const State &xs, const State &xe,
                                         int kDotDeg, int aDotDeg, int N)
  : xs(xs), xe(xe), kDotDeg(kDotDeg), aDotDeg(aDotDeg), N(N) {}

double OptimizationProblem::value(
  const cppoptlib::Problem<double>::TVector &q) {
  Params params(q, kDotDeg, aDotDeg);

  Trajectory traj = shootSimpson(dynamics, xs, params.T, N, params);
  State endpoint = traj.col(traj.cols() - 1);
  double cost = static_cast<double>((xe - endpoint).squaredNorm());
  return cost;
}

}  // namespace Trajectory
}  // namespace PolyTraj