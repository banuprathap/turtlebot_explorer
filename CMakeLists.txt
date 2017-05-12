cmake_minimum_required(VERSION 2.8.3)
project(turtlebot_explorer)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  sensor_msgs
  tf
  geometry_msgs
  actionlib
  move_base
  move_base_msgs
  turtlebot_gazebo
)

###################################
## catkin specific configuration ##
###################################
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES wavefront
  CATKIN_DEPENDS roscpp rospy std_msgs sensor_msgs turtlebot_gazebo move_base_msgs actionlib tf geometry_msgs
)

###########
## Build ##
###########

include_directories(include  ${catkin_INCLUDE_DIRS})

## Declare a C++ library
add_library(wavefront src/wavefront_detection.cpp)
target_link_libraries(wavefront ${catkin_LIBRARIES})

## Declare a cpp executable
add_executable(navigator src/navigator.cpp)
target_link_libraries(navigator ${catkin_LIBRARIES})

## Declare a cpp executable
add_executable(explorer src/explorer.cpp)
target_link_libraries(explorer wavefront)


#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html



#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_turtlebot_explorer.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)

