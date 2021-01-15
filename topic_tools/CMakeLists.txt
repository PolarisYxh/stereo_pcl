cmake_minimum_required(VERSION 3.0.2)
project(my_topic_tools)

if(NOT WIN32)
  set_directory_properties(PROPERTIES COMPILE_OPTIONS "-Wall;-Wextra")
endif()

find_package(catkin COMPONENTS cpp_common message_generation rosconsole roscpp rostime std_msgs xmlrpcpp)

include_directories(${catkin_INCLUDE_DIRS})

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES topic_tools
  CATKIN_DEPENDS message_runtime rosconsole roscpp std_msgs xmlrpcpp
 )

catkin_add_env_hooks(20.transform SHELLS bash DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/env-hooks)
add_library(topic_tools src/shape_shifter.cpp src/parse.cpp)
target_link_libraries(topic_tools ${catkin_LIBRARIES})

add_executable(delay src/delay.cpp)
target_link_libraries(delay topic_tools ${catkin_LIBRARIES})

install(TARGETS delay
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h")