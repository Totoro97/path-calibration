cmake_minimum_required(VERSION 3.13)
project(PathCalibration)

#OpenCV
find_package(OpenCV)
include_directories(${OpenCV_INCLUDE_DIR})

#Eigen
include_directories(/usr/include/eigen3/Eigen)

set(CMAKE_CXX_STANDARD 14)

add_executable(PathCalibration main.cpp)

target_link_libraries(PathCalibration ${OpenCV_LIBS})