cmake_minimum_required(VERSION 3.0.2)
project(open_cv)

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  image_transport
  roscpp
  rospy
  sensor_msgs
  std_msgs
  OpenCV
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES open_cv
#  CATKIN_DEPENDS cv_bridge image_transport roscpp rospy sensor_msgs std_msgs
#  DEPENDS system_lib
)
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

add_executable(eye_sub src/eye_sub.cpp)
target_link_libraries(eye_sub ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})