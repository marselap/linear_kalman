cmake_minimum_required(VERSION 2.8.3)
project(linear_kalman)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    rospy
    roslib
    rospack
)

add_definitions(-std=c++11)

find_package(cmake_modules REQUIRED)

catkin_package(
   INCLUDE_DIRS
   CATKIN_DEPENDS roscpp rospy roslib rospack
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(KalmanFilter src/KalmanFilter.cpp)
target_link_libraries(KalmanFilter ${catkin_LIBRARIES})

add_executable(upsampling_kalman src/KalmanEstimator.cpp)
target_link_libraries(upsampling_kalman KalmanFilter ${catkin_LIBRARIES})