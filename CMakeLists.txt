cmake_minimum_required(VERSION 3.5)

project(lirs_ros_video_streaming)

# Enable c++17
include(CheckCXXCompilerFlag)

CHECK_CXX_COMPILER_FLAG("-std=c++1z" COMPILER_SUPPORTS_CXX17)

if (COMPILER_SUPPORTS_CXX17)
    set(CMAKE_CXX_FLAGS "-std=c++1z -Wall -g")
else ()
    message(FATAL_ERROR "The compiler ${CMAKE_CXX_COMPILER} has no C++17 support. Please use a different C++ compiler.")
endif ()

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
        roscpp
        cv_bridge
        image_transport
        camera_info_manager
        sensor_msgs)

find_package(OpenCV 3 REQUIRED)

catkin_package()

###########
## Build ##
###########

include_directories(include ${catkin_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS})

add_library(v4l2-capture STATIC
        include/lirs_ros_video_streaming/VideoCapture.hpp
        include/lirs_ros_video_streaming/V4L2VideoCapture.hpp
        src/V4L2VideoCapture.cpp)

add_executable(video_streamer src/VideoStreamer.cpp)

target_link_libraries(video_streamer
        ${catkin_LIBRARIES}
        ${OpenCV_LIBS}
        v4l2-capture)

###########
## Test ##
###########

if (CATKIN_ENABLE_TESTING)
    find_package(rostest REQUIRED)
    catkin_add_gtest(v4l2_capture_test test/v4l2_video_capture_test.cpp)
    if (TARGET v4l2_capture_test)
        target_link_libraries(v4l2_capture_test ${catkin_LIBRARIES} v4l2-capture)
    endif()
endif()