#ifndef POLYTRAJ_POLYNOMIAL_H
#define POLYTRAJ_POLYNOMIAL_H

#include <Eigen/Dense>
#include <cstddef>

namespace PolyTraj {

class Polynomial {
public:
  Eigen::VectorXd coeffs;

  explicit Polynomial(const Eigen::VectorXd &coeffs);

  double operator()(double x) const;

  double integral(double x) const;

  double derivative(double x) const;

  long int deg() const;

};

} // namespace PolyTraj
#endif //POLYTRAJ_POLYNOMIAL_H
