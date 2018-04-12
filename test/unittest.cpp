#include <gtest/gtest.h>
#include "path.h"
#include "shoot.h"
#include "trajectory.h"

TEST(polynomial_test, test_poly) {
  Eigen::Vector3d coeffs;
  coeffs << 1.0, 2.0, 3.0;
  polytraj::Polynomial poly(coeffs);

  ASSERT_DOUBLE_EQ(poly(0.0), 1.0);
  ASSERT_DOUBLE_EQ(poly(1.0), 6.0);
  ASSERT_DOUBLE_EQ(poly(2.0), 17.0);
}

TEST(polynomial_test, test_poly_integral) {
  Eigen::Vector3d coeffs;
  coeffs << 1.0, 2.0, 3.0;
  polytraj::Polynomial poly(coeffs);

  ASSERT_DOUBLE_EQ(poly.integral(0.0), 0.0);
  ASSERT_DOUBLE_EQ(poly.integral(1.0), 3.0);
  ASSERT_DOUBLE_EQ(poly.integral(2.0), 14.0);
}

TEST(polynomial_test, test_poly_derivative) {
  Eigen::Vector3d coeffs;
  coeffs << 1.0, 2.0, 3.0;
  polytraj::Polynomial poly(coeffs);

  ASSERT_DOUBLE_EQ(poly.derivative(0.0), 2.0);
  ASSERT_DOUBLE_EQ(poly.derivative(1.0), 8.0);
  ASSERT_DOUBLE_EQ(poly.derivative(2.0), 14.0);
}

TEST(path_test, test_params) {
  using namespace polytraj;

  Eigen::Vector3d coeffs;
  coeffs << 1.0, 2.0, 3.0;
  Polynomial kDotPoly(coeffs);
  auto params = path::Params(5.0, kDotPoly);

  ASSERT_DOUBLE_EQ(params(2.0), 17.0);
}

TEST(path_test, test_optimization) {
  using namespace polytraj;

  path::State xs;
  xs << 1.0, 1.0, M_PI_2, -0.5;
  path::State xe;
  xe << 10.0, 10.0, M_PI_2, 0.5;

  path::Params p = path::optimizeParams(xs, xe);

  ASSERT_NEAR(p.S, 13.347, 1e-3);
  ASSERT_NEAR(p.kDotPoly.coeffs[0], 0.13954400, 1e-3);
  ASSERT_NEAR(p.kDotPoly.coeffs[1], -0.00315048, 1e-3);
  ASSERT_NEAR(p.kDotPoly.coeffs[2], -0.00525480, 1e-3);
  ASSERT_NEAR(p.kDotPoly.coeffs[3], 0.00045160, 1e-3);
}

TEST(path_test, test_simpson) {
  using namespace polytraj;

  path::State xs;
  xs << 0, 0, 0, 0;

  Eigen::Vector4d coeffs;
  coeffs << 1.0, 0.1, -0.1, 0.1;
  path::Params p(10.0, Polynomial(coeffs));

  // Shoot and check endpoint.
  path::Path path = shootSimpson(path::dynamics, xs, p.S, 1001, p);
  path::State end = path.rightCols(1);

  ASSERT_NEAR(end[path::SX], 0.922, 1e-1);
  ASSERT_NEAR(end[path::SY], 0.875, 1e-1);
  ASSERT_NEAR(end[path::ST], 480.255, 1e-1);
  ASSERT_NEAR(end[path::SK], 231.665, 1e-1);
}

TEST(path_test, test_trapezoidal) {
  using polytraj::shootTrapezoidal;
  using polytraj::Polynomial;
  using namespace polytraj::path;

  State xs;
  xs << 0, 0, 0, 0;

  Eigen::Vector4d coeffs;
  coeffs << 1.0, 0.1, -0.1, 0.1;
  Params p(10.0, Polynomial(coeffs));

  // Shoot and check endpoint.
  Path path = shootTrapezoidal(dynamics, xs, p.S, 1001, p);
  State end = path.rightCols(1);

  ASSERT_NEAR(end[SX], 0.926, 1e-1);
  ASSERT_NEAR(end[SY], 0.867, 1e-1);
  ASSERT_NEAR(end[ST], 481.025, 1e-1);
  ASSERT_NEAR(end[SK], 231.667, 1e-1);
}

// TODO: Write more tests for the trajectory generator!
TEST(trajectory_test, test_optimizeParams) {
  using namespace polytraj::trajectory;

  State xs;
  xs << 1.0, 1.0, M_PI_2, -0.5, 1.0, 0.0;
  State xe;
  xe << 10.0, 10.0, M_PI_2, 0.5, 5.0, 0.0;
}
