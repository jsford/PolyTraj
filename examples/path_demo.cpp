/// @example    path_demo.cpp
/// @author     Jordan Ford
/// @brief      Ask the user for a start and end pose and generate a path to connect the two.

#include <polytraj.h>
#include <iostream>
#include <utility>

using namespace polytraj::path;

State ask_user_for_a_state() {
    State s;
    std::cout << "Position  x: "; std::cin >> s[0];
    std::cout << "Position  y: "; std::cin >> s[1];
    std::cout << "Heading   h: "; std::cin >> s[2];
    std::cout << "Curvature k: "; std::cin >> s[3];
    return s;
}

int main() {

  std::cout << "Please Enter a Start Pose:\n";
  State xs = ask_user_for_a_state();
  std::cout << "Please Enter an End Pose:\n";
  State xe = ask_user_for_a_state();

  int num_samples = 0;
  std::cout << "How many samples would you like?\n";
  std::cin  >> num_samples;

  int poly_order = 1;
  std::cout << "What order polynomial would you like?\n";
  std::cin  >> poly_order;

  // Generate 100 points along a path from xs to xe.
  // Use a 4th order polynomial to parameterize the change in steering. (dk/dt)
  Path path = generate(xs, xe, poly_order, num_samples);

  Eigen::IOFormat CleanFmt(4, 0, ", ", "\n", "[", "]");

  std::cout << "Start Posture: " << xs.transpose().format(CleanFmt)
            << std::endl;
  std::cout << "End Posture:   " << xe.transpose().format(CleanFmt)
            << std::endl;
  std::cout << std::endl;

  // Pretty print the path. Each row is a state.
  // The first row is xs. The final row is (nearly) xe.
  std::cout << "Generated Path: [x, y, theta, curvature]" << std::endl;
  std::cout << path.transpose().format(CleanFmt) << std::endl;
}
