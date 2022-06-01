
cmake_minimum_required(VERSION 3.10)
project(game_demo)
set(CMAKE_BUILD_TYPE debug)


SET(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/build)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -Wall -Ofast -Wfatal-errors -D_MWAITXINTRIN_H_INCLUDED")


add_executable(game_demo main_double_buffer.cc)

