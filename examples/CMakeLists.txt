cmake_minimum_required(VERSION 3.0)

# Build the Path example.
add_executable(path_example generate_path.cpp)
target_link_libraries(path_example PolyTrajLib)

# Build the Trajectory example.
add_executable(trajectory_example generate_trajectory.cpp)
target_link_libraries(trajectory_example PolyTrajLib)

# Build the Path demo
add_executable(path_demo path_demo.cpp)
target_link_libraries(path_demo PolyTrajLib)
