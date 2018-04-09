#include "params.h"

namespace PolyTraj {
namespace Trajectory {

Params::Params(double S, const Polynomial &kDotPoly,
               const Polynomial &vDotPoly)
  : S(S), kDotPoly(kDotPoly), vDotPoly(vDotPoly) {}

Params::Params(const Eigen::VectorXd &params, int kDotDegree,
               int vDotDegree)
  : S(params[0]),
    kDotPoly(params.segment(1, kDotDegree)),
    vDotPoly(params.tail(vDotDegree)) {
  assert(kDotDegree + vDotDegree + 1 == params.size());
}

double Params::operator()(double s) const { return kDotPoly(s); }

Eigen::VectorXd Params::vector() const {
  Eigen::VectorXd vec(1 + kDotPoly.deg());
  vec << S, kDotPoly.coeffs;
  return vec;
}

} // namespace Trajectory

namespace Path {

Params::Params(double S, const Polynomial &kDotPoly)
  : S(S), kDotPoly(kDotPoly) {}

Params::Params(const Eigen::VectorXd &params)
  : S(params[0]), kDotPoly(params.tail(params.size() - 1)) {}

double Params::operator()(double s) const { return kDotPoly(s); }

Eigen::VectorXd Params::vector() const {
  Eigen::VectorXd vec(1 + kDotPoly.deg());
  vec << S, kDotPoly.coeffs;
  return vec;
}

} // namespace Path
} // namespace PolyTraj