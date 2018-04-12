cmake_minimum_required(VERSION 3.0)

find_package(GTest REQUIRED)
find_package(Eigen3 3.3 REQUIRED NO_MODULE)

enable_testing()
add_executable(run_tests unittest.cpp)
target_include_directories(run_tests PRIVATE ../src)
target_link_libraries(run_tests PolyTrajLib Eigen3::Eigen GTest::GTest GTest::Main)
add_test(NAME run_tests COMMAND run_tests)

