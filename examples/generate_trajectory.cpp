/// @example    generate_trajectory.cpp
/// @author     Jordan Ford
/// @brief      Generate a trajectory from a start to an end posture.

#include <polytraj.h>
#include <iostream>

int main() {
  using namespace polytraj::trajectory;

  State xs, xe;

  // The trajectory start state.
  xs << 0,     // x = 0
      0,       // y = 0
      M_PI_2,  // theta = pi/2
      0.0,     // curvature = 0
      1.0,     // velocity = 1
      0.0;     // acceleration = 0

  // The trajectory end state.
  xe << 10,    // x = 10
      10,      // y = 10
      M_PI_2,  // theta = pi/2
      0.0,     // curvature = 0
      5.0,     // velocity = 1
      0.0;     // acceleration = 0

  // Generate 100 points along a trajectory from xs to xe.
  // Use a 4th order polynomial to parameterize the change in steering (dk/dt)
  // and a 4th order polynomial to parameterize the change in acceleration
  // (da/dt).
  Trajectory traj = generate(xs, xe, 4, 4, 100);

  Eigen::IOFormat CleanFmt(4, 0, ", ", "\n", "[", "]");

  std::cout << "Start Posture: " << xs.transpose().format(CleanFmt)
            << std::endl;
  std::cout << "End  Posture:  " << xs.transpose().format(CleanFmt)
            << std::endl;
  std::cout << std::endl;

  // Pretty print the trajectory. Each row is a state.
  // The first row is xs. The final row is (nearly) xe.
  std::cout << "Generated Trajectory: [x, y, theta, curvature, velocity, "
               "acceleration]"
            << std::endl;
  std::cout << traj.transpose().format(CleanFmt) << std::endl;
}
