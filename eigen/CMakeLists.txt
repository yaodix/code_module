cmake_minimum_required(VERSION 3.10.0)
project(HELLO)

set(CMAKE_BUILD_TYPE "Debug" CACHE STRING "set build type to debug")
set(CMAKE_CXX_FLAGS "-std=c++11")
# set(CMAKE_BUILD_TYPE "Debug")
# SET(CMAKE_CXX_FLAGS_DEBUG "-O0 -Wall") 

# eigen
find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIRS})

SET(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR})
add_executable(eigen lm_regression.cc)

