cmake_minimum_required(VERSION 3.0.2)
project(lab3)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  kdl_parser
  roscpp
  tf
)


catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES lab3
#  CATKIN_DEPENDS kld_parser roscpp tf
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

add_executable(test_kdl_node src/test_kdl.cpp)

#target_link_libraries(test_node transform_library ${catkin_LIBRARIES} )
#target_link_libraries(publisher_node transform_library ${catkin_LIBRARIES} )
target_link_libraries(test_kdl_node  ${catkin_LIBRARIES} )
