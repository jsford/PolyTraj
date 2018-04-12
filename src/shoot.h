#ifndef POLYTRAJ_SHOOT_H
#define POLYTRAJ_SHOOT_H

#include <Eigen/Dense>
#include <functional>
#include <utility>

namespace PolyTraj {

template <class Fun, class... Args>
inline Eigen::MatrixXd shootTrapezoidal(Fun &func, const Eigen::VectorXd &xs,
                                        double T, int panels, Args &&... args) {
  double h = T / static_cast<double>(panels);

  Eigen::VectorXd f0 = func(0.0, xs, std::forward<Args...>(args...));

  Eigen::MatrixXd path = Eigen::MatrixXd::Zero(4, panels + 1);
  path.col(0) = xs;

  for (int i = 1; i < panels + 1; ++i) {
    Eigen::VectorXd f1 =
        func(i * h, path.col(i - 1), std::forward<Args...>(args...));
    path.col(i) = path.col(i - 1) + (f0 + f1) * h / 2.0;
    f0 = f1;
  }
  return path;
};

template <class Fun, class... Args>
inline Eigen::MatrixXd shootSimpson(Fun func, const Eigen::VectorXd &xs,
                                    double T, int panels, Args &&... args) {
  double h = T / static_cast<double>(panels);

  Eigen::MatrixXd path = Eigen::MatrixXd::Zero(xs.size(), panels + 1);
  Eigen::VectorXd f0, f1, f2;
  f0 = func(0.0, xs, args...);
  path.col(0) = xs;

  for (int i = 1; i < panels; i += 2) {
    f1 = func(i * h, path.col(i - 1), std::forward<Args...>(args...));
    f2 = func(i * h + h, path.col(i - 1), std::forward<Args...>(args...));

    path.col(i) = path.col(i - 1) + (f0 + f1) * (h / 2.0);
    path.col(i + 1) = path.col(i - 1) + (f0 + 4 * f1 + f2) * (h / 3.0);

    f0 = f2;
  }

  if (panels % 2 == 1) {
    f1 = func((panels - 1) * h, path.col(panels - 1), args...);
    path.col(panels) = path.col(panels - 1) + (f0 + f1) * (h / 2.0);
  }

  return path;
}

}  // namespace PolyTraj
#endif  // POLYTRAJ_SHOOT_H
