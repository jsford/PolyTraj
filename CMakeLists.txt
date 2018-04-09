cmake_minimum_required(VERSION 3.0)
project(PolyTraj)

set(CMAKE_CXX_STANDARD 14)

# Find external libs.
find_package(GTest REQUIRED)
find_package(Eigen3 3.3 REQUIRED NO_MODULE)

# Pull in included external libs.
add_subdirectory(external)

# Build PolyTraj library.
add_library(PolyTrajLib

        src/path.h
        src/path.cpp

        src/trajectory.h
        src/trajectory.cpp

        src/shoot.h

        src/polynomial.h
        src/polynomial.cpp

        src/params.h
        src/params.cpp

        include/polytraj.h
        )

# Don't like making internal headers public...
target_include_directories(PolyTrajLib PUBLIC include)
target_include_directories(PolyTrajLib PRIVATE src)
target_link_libraries(PolyTrajLib Eigen3::Eigen CppOptLib)

# Build examples.
add_executable(run_examples examples/main.cpp)
target_link_libraries(run_examples PolyTrajLib Eigen3::Eigen)

# Build test executable.
enable_testing()
add_executable(run_tests test/unittest.cpp)
target_include_directories(run_tests PRIVATE src)
target_link_libraries(run_tests PolyTrajLib Eigen3::Eigen GTest::GTest GTest::Main)
add_test(NAME run_tests COMMAND run_tests)
