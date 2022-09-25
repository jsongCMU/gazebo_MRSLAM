cmake_minimum_required(VERSION 3.0.2)
project(robosar_gazebo_sim)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS roscpp std_msgs tf geometry_msgs apriltag_ros)

catkin_package(
  CATKIN_DEPENDS roscpp std_msgs tf geometry_msgs apriltag_ros roslib
)

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(tag_location_estimator src/tag_location_estimator.cpp)
target_link_libraries(tag_location_estimator ${catkin_LIBRARIES})

add_executable(apriltag_publisher_gazebo src/apriltag_publisher.cpp)
target_link_libraries(apriltag_publisher_gazebo ${catkin_LIBRARIES})
