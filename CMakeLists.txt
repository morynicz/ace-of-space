cmake_minimum_required(VERSION 2.8)
project(AceOfSpace CXX C)

find_package(OpenCV REQUIRED)
find_package(PCL 1.3 REQUIRED)

set(CMAKE_BUILD_TYPE Debug)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

include_directories(inc)

add_executable(test src/main.cpp src/ConvenienceFunctions.cpp)
target_link_libraries(test opencv_core opencv_highgui opencv_features2d opencv_calib3d opencv_nonfree opencv_imgproc)