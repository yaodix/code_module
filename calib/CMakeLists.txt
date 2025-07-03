cmake_minimum_required(VERSION 3.10.0)
project(HELLO)

set(CMAKE_BUILD_TYPE "Debug" CACHE STRING "set build type to debug")
set(CMAKE_CXX_FLAGS "-std=c++11")
# set(CMAKE_BUILD_TYPE "Debug")
# SET(CMAKE_CXX_FLAGS_DEBUG "-O0 -Wall") 

# eigen
find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIRS})

#pcl
find_package(PCL REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})

#opencv
find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})


add_executable(proj lidar_proj_cam.cpp)
target_link_libraries(proj ${OpenCV_LIBS} ${PCL_LIBRARIES})
