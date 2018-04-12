#include <polytraj.h>

#include <iostream>

int main() {
  using namespace polytraj::trajectory;

  State xs, xe;
  xs << 0, 0, M_PI_2, 0.0, 1.0, 0.0;
  xe << 10, 10, M_PI_2, 0.0, 5.0, 0.0;

  Trajectory traj = generate(xs, xe, 4, 4, 100);

  Eigen::IOFormat CleanFmt(4, 0, ", ", "\n", "[", "]");
  std::cout << traj.transpose().format(CleanFmt) << std::endl;
}
