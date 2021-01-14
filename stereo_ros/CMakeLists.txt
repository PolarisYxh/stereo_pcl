cmake_minimum_required(VERSION 3.0.2)
project(stereo_ros)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)
ADD_DEFINITIONS(-std=c++11 -fPIC -DENABLE_SSE -D__DEBUG -msse -msse2 -msse3 -mssse3 -msse4.1 -msse4.2)
find_package(catkin REQUIRED COMPONENTS roscpp sensor_msgs)
set(OpenCV_DIR /opt/ros/kinetic/share/OpenCV-3.3.1)
find_package(OpenCV)

FIND_PACKAGE( PCL REQUIRED )
list(REMOVE_ITEM PCL_LIBRARIES "vtkproj4") # use this in Ubuntu 16.04

ADD_DEFINITIONS( ${PCL_DEFINITIONS} )
INCLUDE_DIRECTORIES( ${PCL_INCLUDE_DIRS}  )
LINK_LIBRARIES( ${PCL_LIBRARY_DIRS} )

include_directories(${OPENCV_INCLUDE_DIRS})
find_package(catkin REQUIRED COMPONENTS
  roscpp
  cv_bridge
  tf
)



catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES stereo_ros
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include 
  ${catkin_INCLUDE_DIRS}
# include
# ${catkin_INCLUDE_DIRS}
)

include_directories(src)

# FILE(GLOB LIBELAS_SRC_FILES "src/*.cpp")
# add_library(elaslib 
#   src/descriptor.cpp
#   src/elas.cpp
#   src/filter.cpp
#   src/matrix.cpp
#   src/triangle.cpp )

add_executable(stereo_ros_pub
  src/stereo_ros_pub.cpp
)
add_dependencies(stereo_ros_pub ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(stereo_ros_pub
  ${catkin_LIBRARIES}
)

add_executable(stereo_ros_sub
  src/stereo_ros_sub.cpp
    src/descriptor.cpp
  src/elas.cpp
  src/filter.cpp
  src/matrix.cpp
  src/triangle.cpp
)
add_dependencies(stereo_ros_sub ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(stereo_ros_sub 
  ${OpenCV_LIBRARIES}
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
  # elaslib
)