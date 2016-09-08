cmake_minimum_required(VERSION 2.8.3)
project(pioneer_delivery)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  rospy
)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(delivery_waypoint src/delivery_waypoint.cpp)
target_link_libraries(delivery_waypoint ${catkin_LIBRARIES})
add_dependencies(delivery_waypoint ${catkin_EXPORTED_TARGETS})
