cmake_minimum_required(VERSION 3.0.2)
project(sparse_ros)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp sensor_msgs)
FIND_PACKAGE( PCL REQUIRED )
list(REMOVE_ITEM PCL_LIBRARIES "vtkproj4") # use this in Ubuntu 16.04

ADD_DEFINITIONS( ${PCL_DEFINITIONS} )
INCLUDE_DIRECTORIES( ${PCL_INCLUDE_DIRS}  )
LINK_LIBRARIES( ${PCL_LIBRARY_DIRS} )


##   * add "message_generation" and every package in MSG_DEP_SET to
find_package(catkin REQUIRED COMPONENTS
  roscpp
)
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES sparse_ros
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include ${catkin_INCLUDE_DIRS}
# include
# ${catkin_INCLUDE_DIRS}
)



add_executable(sparse_ros_pub
  src/sparse_ros_pub.cpp
)
add_dependencies(sparse_ros_pub ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(sparse_ros_pub
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)

add_executable(sparse_ros_sub
  src/sparse_ros_sub.cpp
)
add_dependencies(sparse_ros_sub ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(sparse_ros_sub
  ${catkin_LIBRARIES}
)