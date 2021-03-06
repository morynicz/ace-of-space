cmake_minimum_required(VERSION 2.8)
project(AceOfSpace CXX C)

if(UNIX)
    SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -std=c++11")
endif()

find_package(OpenCV REQUIRED)
find_package(PCL 1.3 REQUIRED)
find_package(Boost REQUIRED program_options system)
find_package(Threads)

set(CMAKE_BUILD_TYPE Debug)

include_directories(inc)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})



#add_library(catching src/Catcher.cpp)

add_executable(tests src/main.cpp src/ConvenienceFunctions.cpp src/StorageFunctions.cpp)
target_link_libraries(tests opencv_core opencv_highgui opencv_features2d
  opencv_calib3d opencv_stereo opencv_imgproc opencv_xfeatures2d ${Boost_LIBRARIES} ${PCL_LIBRARIES})

add_executable(prepareCalib
	src/stereoCalibPrep.cpp
	src/ConvenienceFunctions.cpp
	src/StorageFunctions.cpp
	src/Catcher.cpp)
target_link_libraries(prepareCalib opencv_core opencv_highgui opencv_imgproc
  ${Boost_LIBRARIES} ${CMAKE_THREAD_LIBS_INIT})

add_executable(editPairList
	src/editStereoList.cpp
	src/ConvenienceFunctions.cpp
	src/StorageFunctions.cpp)
target_link_libraries(editPairList opencv_core opencv_highgui opencv_calib3d
  ${Boost_LIBRARIES} ${CMAKE_THREAD_LIBS_INIT})

add_executable(calibrateCams
	src/StereoCalib.cpp
	src/ConvenienceFunctions.cpp
	src/StorageFunctions.cpp)
target_link_libraries(calibrateCams opencv_core opencv_highgui opencv_calib3d opencv_imgproc
  ${Boost_LIBRARIES} ${CMAKE_THREAD_LIBS_INIT})

if(UNIX)
  set_target_properties(prepareCalib PROPERTIES COMPILE_FLAGS ${COMPILE_FLAGS} -std=c++11)
  set(CXX_FLAGS ${CXX_FLAGS} -Wall)
  set_target_properties(tests PROPERTIES COMPILE_FLAGS ${COMPILE_FLAGS}-std=c++11)
#  set_target_properties(catching PROPERTIES COMPILE_FLAGS ${COMPILE_FLAGS} -std=c++11)
endif(UNIX)