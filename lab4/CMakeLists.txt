cmake_minimum_required(VERSION 3.0.2)
project(lab4)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  kdl_parser
)

catkin_package()

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

add_executable(command_pub_node src/command_pub.cpp)

#target_link_libraries(test_node transform_library ${catkin_LIBRARIES} )
#target_link_libraries(publisher_node transform_library ${catkin_LIBRARIES} )
target_link_libraries(command_pub_node  ${catkin_LIBRARIES} )
