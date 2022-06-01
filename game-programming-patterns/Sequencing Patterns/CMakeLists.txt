
cmake_minimum_required(VERSION 3.10)
project(game_demo)
set(CMAKE_BUILD_TYPE debug)


SET(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/build)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14 -Wall -O0 -Wfatal-errors -D_MWAITXINTRIN_H_INCLUDED")

find_package(OpenCV 4.5.4 REQUIRED) #找OpenCV 4.0安装路径
include_directories(${OpenCV_INCLUDE_DIRS}) #加载OpenCV 4.0的头文件


add_executable(game_demo away_from_white_block.cc)

target_link_libraries(game_demo ${OpenCV_LIBS}) #链接lib文件到可执行文件中