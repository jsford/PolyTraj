#include <path.h>

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

Path shootTrapezoidal(const PathState &xs, const PathParams &params,
                      int N = 100) {
  double S = params.S;
  double h = S / static_cast<double>(N);

  PathState f0 = dynamics(0.0, xs, params);

  Path path = Path::Zero(4, N + 1);
  path.col(0) = xs;

  for (int i = 1; i < N + 1; ++i) {
    PathState f1 = dynamics(i * h, path.col(i - 1), params);
    path.col(i) = path.col(i - 1) + (f0 + f1) * h / 2.0;
    f0 = f1;
  }
  return path;
};

Path shootSimpson(const PathState &xs, const PathParams &params, int N = 100) {
  double h = params.S / static_cast<double>(N);

  Path path = Path::Zero(xs.size(), N + 1);
  PathState f0, f1, f2;
  f0 = dynamics(0.0, xs, params);
  path.col(0) = xs;

  for (int i = 1; i < N; i += 2) {
    f1 = dynamics(i * h, path.col(i - 1), params);
    f2 = dynamics(i * h + h, path.col(i - 1), params);

    path.col(i) = path.col(i - 1) + (f0 + f1) * (h / 2.0);
    path.col(i + 1) = path.col(i - 1) + (f0 + 4 * f1 + f2) * (h / 3.0);

    f0 = f2;
  }

  if (N % 2 == 1) {
    f1 = dynamics((N - 1) * h, path.col(N - 1), params);
    path.col(N) = path.col(N - 1) + (f0 + f1) * (h / 2.0);
  }

  return path;
}

class PathOptimizationProblem : public cppoptlib::Problem<double> {
public:
  using typename cppoptlib::Problem<double>::TVector;
  using typename cppoptlib::Problem<double>::THessian;

  PathState xs;
  PathState xe;
  int N;

  PathOptimizationProblem(const PathState &xs, const PathState &xe, int N = 100)
    : xs(xs), xe(xe), N(N) {}

  double value(const TVector &q) {
    PathParams params(q);
    Path path = shootSimpson(xs, params);
    PathState endpoint = path.col(path.cols() - 1);
    double cost = (xe - endpoint).squaredNorm();
    return cost;
  }
  // void gradient(const TVector& q, TVector& grad) {}
  // void hessian(const TVector& q, THessian& hess) {}
};

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

  return shootSimpson(initialState, params, points - 1);
}

}  // namespace PolyTraj
