

cmake_minimum_required(VERSION 3.14...3.19 FATAL_ERROR)

project("learn_eigen")
set(CMAKE_BUILD_TYPE debug)
SET(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR})


include_directories("myPackage/Package")

# OpenCV
message(STATUS "ZHITO_THIRD_PARTY_DIR:${ZHITO_THIRD_PARTY_DIR}")
set(OpenCV_DIR "${ZHITO_THIRD_PARTY_DIR}/OpenCV320/share/OpenCV")
find_package(OpenCV 3.2 REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})
message(STATUS "OpenCV version: "${OpenCV_VERSION})
message(STATUS "OpenCV_LIBS:${OpenCV_LIBS}")
message(STATUS "OpenCV_INCLUDE_DIRS:${OpenCV_INCLUDE_DIRS}")

# Eigen
find_package(Eigen3 REQUIRED)
include_directories(SYSTEM ${EIGEN3_INCLUDE_DIR})
message(STATUS "EIGEN3_VERSION: ${EIGEN3_VERSION}")

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14 -Wall -O0 -Wfatal-errors -D_MWAITXINTRIN_H_INCLUDED")

add_executable(lm_regression lm_regression.cc)
target_link_libraries(eigen ${OpenCV_LIBS})