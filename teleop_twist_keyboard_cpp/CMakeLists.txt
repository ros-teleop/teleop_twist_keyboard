cmake_minimum_required(VERSION 2.8.3)
project(teleop_twist_keyboard_cpp)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  geometry_msgs
)

catkin_package(
  INCLUDE_DIRS src
  CATKIN_DEPENDS roscpp geometry_msgs
)

include_directories(${catkin_INCLUDE_DIRS})

add_executable(teleop_twist_keyboard src/teleop_twist_keyboard.cpp)
target_link_libraries(teleop_twist_keyboard ${catkin_LIBRARIES})

set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")
