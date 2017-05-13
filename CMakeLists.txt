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
target_link_libraries(explorer wavefront ${catkin_LIBRARIES})


add_library(src_navigator src/navigator.cpp)
target_link_libraries(src_navigator ${catkin_LIBRARIES})

add_library(src_explorer src/explorer.cpp)
target_link_libraries(src_explorer wavefront ${catkin_LIBRARIES})


#############
## Testing ##
#############

if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  ###########  EXPLORER TESTS  ###########
  add_rostest_gtest(
  test_explorer
  test/test_explorer.launch
  test/test_explorer.cpp
  )
  target_link_libraries(
  test_explorer
  wavefront
  src_explorer
  ${catkin_LIBRARIES}
  )
  add_dependencies(
  test_explorer
  #explorer
  ${catkin_EXPORTED_TARGETS}
  )
  ###########  NAVIGATOR TESTS  ###########
  add_rostest_gtest(
  test_navigator
  test/test_navigator.launch
  test/test_navigator.cpp
  #src/navigator.cpp
  )
  target_link_libraries(
  test_navigator
  src_navigator
  #wavefront
  ${catkin_LIBRARIES}
  )
  add_dependencies(
  test_navigator
  #navigator
  ${catkin_EXPORTED_TARGETS}
  )
endif()


