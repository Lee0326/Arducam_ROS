cmake_minimum_required(VERSION 2.8.3)
add_compile_options(-std=c++11)
project(arducam_publisher)

SET(CMAKE_CXX_COMPILER "g++") 

set( CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread" )


find_package(catkin REQUIRED COMPONENTS cv_bridge image_transport message_generation sensor_msgs roscpp)

catkin_package(CATKIN_DEPENDS cv_bridge image_transport message_runtime sensor_msgs roscpp)

find_package(OpenCV) 

include_directories(${catkin_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS})


add_executable(arducam_publisher src/capture_ac_ros.cpp)
target_link_libraries(arducam_publisher ${catkin_LIBRARIES} ${OpenCV_LIBS})

