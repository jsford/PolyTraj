#include "path.h"
#include "shoot.h"

#include <cppoptlib/meta.h>
#include <cppoptlib/problem.h>
#include <cppoptlib/solver/neldermeadsolver.h>
#include <cppoptlib/solver/newtondescentsolver.h>

namespace PolyTraj {

PathState dynamics(const double s, const PathState &x,
                   const PathParams &params) {
  PathState xDot;
  xDot << std::cos(x[PST]), std::sin(x[PST]), x[PSK], params.kDotPoly(s);
  return xDot;
}

PathOptimizationProblem::PathOptimizationProblem(const PathState &xs,
                                                 const PathState &xe, int N)
    : xs(xs), xe(xe), N(N) {}


double PathOptimizationProblem::value(const TVector &q) {
  PathParams params(q);

  Path path = shootSimpson(dynamics, xs, params.S, N, params);
  PathState endpoint = path.col(path.cols() - 1);
  double cost = static_cast<double>((xe - endpoint).squaredNorm());
  return cost;
}

PathParams initPathParams(const PathState &xs, const PathState &xe) {
  const double dx = xe[PSX] - xs[PSX];
  const double dy = xe[PSY] - xs[PSY];
  const double dt = xe[PST] - xs[PST];

  const double R = std::sqrt(dx * dx + dy * dy);
  const double S = (0.2 * dt * dt + 1.0) * R;

  const double c0 = 6 * xe[PST] / (S * S) - 4 * xs[PSK] / S - 2 * xe[PSK] / S;
  const double c1 =
      6 * (xs[PSK] + xe[PSK]) / (S * S) - 6 * xe[PST] / (S * S * S);
  const double c2 = 0;
  const double c3 = 0;

  Eigen::VectorXd coeffs(4);
  coeffs << c0, c1, c2, c3;
  Polynomial kDotPoly(coeffs);

  return PathParams(S, kDotPoly);
}

PathParams optimizePath(const PathState &xs, const PathState &xe) {
  PathParams initParams = initPathParams(xs, xe);

  cppoptlib::NelderMeadSolver<PathOptimizationProblem> solver;
  PathOptimizationProblem f(xs, xe);

  Eigen::VectorXd p = initParams.vector();
  solver.minimize(f, p);
  return PathParams(p);
}

Path generatePath(const PathState &initialState, const PathState &finalState,
                  int points) {
  PathParams params = optimizePath(initialState, finalState);

  return shootSimpson(dynamics, initialState, params.S, points - 1, params);
}

}  // namespace PolyTraj
