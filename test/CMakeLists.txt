cmake_minimum_required(VERSION 3.0.2)
project(test)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  tf
)

catkin_package()

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

#add_executable(test_node src/test_node.cpp)
#add_executable(publisher_node src/publisher_node.cpp)
add_executable(lab2_node src/lab2_node.cpp include/HomogeneousTransformationClass.cpp)

#target_link_libraries(test_node transform_library ${catkin_LIBRARIES} )
#target_link_libraries(publisher_node transform_library ${catkin_LIBRARIES} )
target_link_libraries(lab2_node  ${catkin_LIBRARIES} )
