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

# Build the PolyTrajLib target.
add_subdirectory(src)

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
