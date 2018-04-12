#include <polynomial.h>

namespace polytraj {

Polynomial::Polynomial(const Eigen::VectorXd &coeffs) : coeffs(coeffs) {}

double Polynomial::operator()(double x) const {
  double y = coeffs[coeffs.size() - 1];
  for (long int i = coeffs.size() - 2; i >= 0; --i) {
    y = x * y + coeffs[i];
  }
  return y;
}

double Polynomial::integral(double x) const {
  double y = coeffs[coeffs.size() - 1] / (float) coeffs.size();

  for (long int i = coeffs.size() - 2; i >= 0; --i) {
    y = x * y + coeffs[i] / (float) (i + 1);
  }
  return x * y;
}

double Polynomial::derivative(double x) const {
  double y = coeffs[coeffs.size() - 1] * (float) (coeffs.size() - 1);

  for (long int i = coeffs.size() - 2; i >= 1; --i) {
    y = x * y + coeffs[i] * (float) (i);
  }
  return y;
}

long int Polynomial::deg() const {
  return coeffs.size();
}

} // namespace polytraj
