cmake_minimum_required(VERSION 3.0.2)
project(robosar_gazebo_sim)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS roscpp std_msgs tf geometry_msgs)

catkin_package()

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(tag_location_estimator src/tag_location_estimator.cpp)
target_link_libraries(tag_location_estimator ${catkin_LIBRARIES})

