cmake_minimum_required(VERSION 3.10.0)
project(HELLO)

# set(CMAKE_BUILD_TYPE "Debug" CACHE STRING "set build type to debug")
set(CMAKE_CXX_STANDARD 14)
# set(CMAKE_BUILD_TYPE "Debug")
# SET(CMAKE_CXX_FLAGS_DEBUG "-O0 -Wall") 

# OpenCV
message(STATUS "ZHITO_THIRD_PARTY_DIR:${ZHITO_THIRD_PARTY_DIR}")
set(OpenCV_DIR "${ZHITO_THIRD_PARTY_DIR}/OpenCV320/share/OpenCV")
find_package(OpenCV 3.2 REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})
message(STATUS "OpenCV version: "${OpenCV_VERSION})
message(STATUS "OpenCV_LIBS:${OpenCV_LIBS}")
message(STATUS "OpenCV_INCLUDE_DIRS:${OpenCV_INCLUDE_DIRS}")

# eigen
find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIRS})
# 定义自己编译的Ceres的库目录
# set(Ceres_LIBS_DIR /usr/local/ceres/lib)

add_subdirectory(ceres)
include_directories(/usr/local/ceres//include)


SET(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR})

add_executable(ceres_demo demo_01.cc)
target_link_libraries(ceres_demo ceres)

add_executable(fitting_epx fitting_epx.cc)
target_link_libraries(fitting_epx ceres ${OpenCV_LIBS})
