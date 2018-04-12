/// @example    generate_path.cpp
/// @author     Jordan Ford
/// @brief      Generate a path from a start to an end posture.

#include <polytraj.h>
#include <iostream>

int main() {
  using namespace polytraj::path;

  State xs, xe;

  // The path start state.
  xs << 0,          // x = 0
        0,          // y = 0
        M_PI_2,     // theta = pi/2
        0.0;        // curvature = 0

  // The path end state.
  xe << 10,         // x = 10
        10,         // y = 10
        M_PI_2,     // theta = pi/2
        0.0;        // curvature = 0

  // Generate 100 points along a path from xs to xe.
  // Use a 4th order polynomial to parameterize the change in steering. (dk/dt)
  Path path = generate(xs, xe, 4, 100);

  // Pretty print the path. Each row is a state.
  // The first row is xs. The final row is (nearly) xe.
  Eigen::IOFormat CleanFmt(4, 0, ", ", "\n", "[", "]");
  std::cout << path.transpose().format(CleanFmt) << std::endl;
}
