#include "params.h"

namespace PolyTraj {

TrajParams::TrajParams(double S, const Polynomial &kDotPoly,
                       const Polynomial &vDotPoly)
  : S(S), kDotPoly(kDotPoly), vDotPoly(vDotPoly) {}

TrajParams::TrajParams(const Eigen::VectorXd &params, int kDotDegree,
                       int vDotDegree)
  : S(params[0]),
    kDotPoly(params.segment(1, kDotDegree)),
    vDotPoly(params.tail(vDotDegree)) {
  assert(kDotDegree + vDotDegree + 1 == params.size());
}

double TrajParams::operator()(double s) const { return kDotPoly(s); }

Eigen::VectorXd TrajParams::vector() const {
  Eigen::VectorXd vec(1 + kDotPoly.deg());
  vec << S, kDotPoly.coeffs;
  return vec;
}

PathParams::PathParams(double S, const Polynomial &kDotPoly)
  : S(S), kDotPoly(kDotPoly) {}

PathParams::PathParams(const Eigen::VectorXd &params)
  : S(params[0]), kDotPoly(params.tail(params.size() - 1)) {}

double PathParams::operator()(double s) const { return kDotPoly(s); }

Eigen::VectorXd PathParams::vector() const {
  Eigen::VectorXd vec(1 + kDotPoly.deg());
  vec << S, kDotPoly.coeffs;
  return vec;
}
}  // namespace PolyTraj