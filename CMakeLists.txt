cmake_minimum_required(VERSION 3.13)
project(PathCalibration)
set(CMAKE_BUILD_TYPE Release)
set(CMAKE_CXX_STANDARD 14)

#OpenCV
find_package(OpenCV)
include_directories(${OpenCV_INCLUDE_DIR})

#Eigen
include_directories(/usr/include/eigen3/Eigen)


set(SOURCE_FILES src/Algorithms.cpp src/Algorithms.h)
set(SOURCE_FILES ${SOURCE_FILES} src/Calibrator.cpp src/Calibrator.h)
set(SOURCE_FILES ${SOURCE_FILES} src/GlobalCalibrator.cpp src/GlobalCalibrator.h)
set(SOURCE_FILES ${SOURCE_FILES} src/DistMap.cpp src/DistMap.h)
set(SOURCE_FILES ${SOURCE_FILES} src/ViewTransformer.cpp src/ViewTransformer.h)
set(SOURCE_FILES ${SOURCE_FILES} src/Utils.cpp src/Utils.h)
set(SOURCE_FILES ${SOURCE_FILES} src/Thinning.h)

add_executable(PathCalibration ${SOURCE_FILES} src/Main.cpp)

target_link_libraries(PathCalibration ${OpenCV_LIBS})