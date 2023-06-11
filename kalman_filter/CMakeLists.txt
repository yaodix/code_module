cmake_minimum_required(VERSION 3.10)
project(kalman_filter)

set(CMAKE_BUILD_TYPE "Debug" CACHE STRING "set build type to debug")
set(CMAKE_CXX_FLAGS "-std=c++11")
# set(CMAKE_BUILD_TYPE "Debug")
# SET(CMAKE_CXX_FLAGS_DEBUG "-O0 -Wall") 

# eigen
find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIRS})

#opencv
find_package(OpenCV 4.5.4 REQUIRED) #找OpenCV 4.0安装路径
include_directories(${OpenCV_INCLUDE_DIRS}) #加载OpenCV 4.0的头文件


SET(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR})
add_executable(kalman_filter test_kalman_filter.cc kalman_filter.cc)