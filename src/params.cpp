#include "params.h"

namespace polytraj {
namespace trajectory {

Params::Params(double T, const Polynomial &kDotPoly,
               const Polynomial &aDotPoly)
  : T(T), kDotPoly(kDotPoly), aDotPoly(aDotPoly) {}

Params::Params(const Eigen::VectorXd &params, int kDotDegree,
               int vDotDegree)
  : T(params[0]),
    kDotPoly(params.segment(1, kDotDegree)),
    aDotPoly(params.tail(vDotDegree)) {
  assert(kDotDegree + vDotDegree + 1 == params.size());
}

double Params::operator()(double s) const { return kDotPoly(s); }

Eigen::VectorXd Params::vector() const {
  Eigen::VectorXd vec(1 + kDotPoly.deg() + aDotPoly.deg());
  vec << T, kDotPoly.coeffs, aDotPoly.coeffs;
  return vec;
}

} // namespace trajectory

namespace path {

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

} // namespace path
} // namespace polytraj