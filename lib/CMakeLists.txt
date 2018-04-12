cmake_minimum_required(VERSION 3.0)

# Find necessary libs.
find_package(Eigen3 3.3 REQUIRED NO_MODULE)

# Pull in included external libs.
add_subdirectory(${PROJECT_SOURCE_DIR}/external ${CMAKE_BINARY_DIR}/external)

# Build polytraj library.
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

# Grab headers from the include directory.
target_include_directories(PolyTrajLib PUBLIC include)

# Grab headers from this src directory. Don't share them with clients.
target_include_directories(PolyTrajLib PRIVATE src)

# Pull in Eigen for linear algebra.
# Pull in CppOptLib for optimization.
target_link_libraries(PolyTrajLib Eigen3::Eigen CppOptLib)

# Set options for the compiler.
target_compile_options(PolyTrajLib PRIVATE -Wall)
