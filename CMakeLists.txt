cmake_minimum_required(VERSION 3.0)
project(PolyTraj)

# Use C++ 14
set(CMAKE_CXX_STANDARD 14)

# Select what to build.
option(BUILD_SHARED_LIBS   "Build as shared library." OFF)
option(BUILD_EXAMPLES      "Build example programs."   ON)
option(BUILD_TESTS         "Build test programs."      ON)
option(BUILD_DOCS          "Build documentation."      ON)

# Set Version Info
set(VERSION_MAJOR   0   CACHE STRING "Project major version number.")
set(VERSION_MINOR   0   CACHE STRING "Project minor version number.")
set(VERSION_PATCH   0   CACHE STRING "Project patch version number.")
mark_as_advanced(VERSION_MAJOR VERSION_MINOR VERSION_PATCH)


# Find necessary libs.
find_package(Eigen3 3.3 REQUIRED NO_MODULE)

# Pull in included external libs.
add_subdirectory(external)

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

target_include_directories(PolyTrajLib PUBLIC include)
target_include_directories(PolyTrajLib PRIVATE src)
target_link_libraries(PolyTrajLib Eigen3::Eigen CppOptLib)
target_compile_options(PolyTrajLib PRIVATE -Wall)

# Build examples.
if(BUILD_EXAMPLES)
    add_subdirectory(examples)
endif(BUILD_EXAMPLES)

# Build test executable.
if(BUILD_TESTS)
    add_subdirectory(test)
endif(BUILD_TESTS)

# Build doxygen documentation.
if(BUILD_DOCS)
    add_subdirectory(docs)
    add_dependencies(doc PolyTrajLib)
endif(BUILD_DOCS)
