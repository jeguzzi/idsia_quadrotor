#Derived from the joypad_flying package by rpg
#Mix input from /assisted_cmd topic and joystick

cmake_minimum_required(VERSION 2.8.3)
project(assisted_joypad_flying)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  sensor_msgs
  quad_common
  quad_msgs
  copilot
  flyingroom
)
find_package(Eigen REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp std_msgs sensor_msgs quad_common quad_msgs copilot flyingroom forest_msgs
  DEPENDS eigen
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN_INCLUDE_DIRS}
)

add_executable(assisted_joypad_flying
  src/assisted_joypad_flying.cpp
)

add_dependencies(assisted_joypad_flying  ${catkin_EXPORTED_TARGETS})

target_link_libraries(assisted_joypad_flying
   ${catkin_LIBRARIES}
 )

add_dependencies(assisted_joypad_flying forest_msgs_generate_messages_cpp)
 


