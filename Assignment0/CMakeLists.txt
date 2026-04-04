cmake_minimum_required (VERSION 3.10)
project (Transformation)

find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIRS})

add_executable (Transformation main.cpp)

# Use C++14 as in the examples
set_target_properties(Transformation PROPERTIES CXX_STANDARD 14 CXX_STANDARD_REQUIRED ON)
