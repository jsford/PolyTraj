cmake_minimum_required(VERSION 3.0)

find_package(Eigen3 3.3 REQUIRED NO_MODULE)
add_executable(run_examples main.cpp)
target_link_libraries(run_examples PolyTrajLib Eigen3::Eigen)
