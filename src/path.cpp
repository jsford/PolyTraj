#include "path.h"
#include "shoot.h"

#include <cppoptlib/solver/neldermeadsolver.h>
#include <cppoptlib/solver/newtondescentsolver.h>

namespace PolyTraj {
namespace Path {

State dynamics(const double s, const State &x, const Params &params) {
  State xDot;
  xDot << std::cos(x[ST]), std::sin(x[ST]), x[SK], params.kDotPoly(s);
  return xDot;
}

OptimizationProblem::OptimizationProblem(const State &xs, const State &xe,
                                         int N)
  : xs(xs), xe(xe), N(N) {}

double OptimizationProblem::value(const TVector &q) {
  Params params(q);

  Path path = shootSimpson(dynamics, xs, params.S, N, params);
  State endpoint = path.col(path.cols() - 1);
  double cost = static_cast<double>((xe - endpoint).squaredNorm());
  return cost;
}

Params initParams(const State &xs, const State &xe, int kDotDeg) {
  assert(kDotDeg >= 2);

  double dx = xe[SX] - xs[SX];
  double dy = xe[SY] - xs[SY];
  double dt = xe[ST] - xs[ST];

  double R = std::sqrt(dx * dx + dy * dy);
  double S = (0.2 * dt * dt + 1.0) * R;

  Eigen::VectorXd coeffs = Eigen::VectorXd::Zero(kDotDeg);
  coeffs[0] = 6 * xe[ST] / (S * S) - 4 * xs[SK] / S - 2 * xe[SK] / S;
  coeffs[1] = 6 * (xs[SK] + xe[SK]) / (S * S) - 6 * xe[ST] / (S * S * S);

  Polynomial kDotPoly(coeffs);

  return Params(S, kDotPoly);
}

Params optimizeParams(const State &xs, const State &xe, int kDotDeg) {
  Params params = initParams(xs, xe, kDotDeg);

  cppoptlib::NelderMeadSolver<OptimizationProblem> solver;
  OptimizationProblem f(xs, xe);

  Eigen::VectorXd p = params.vector();
  solver.minimize(f, p);
  return Params(p);
}

Path generate(const State &initialState, const State &finalState, int points) {
  Params params = optimizeParams(initialState, finalState);

  return shootSimpson(dynamics, initialState, params.S, points - 1, params);
}

}  // namespace Path
}  // namespace PolyTraj
