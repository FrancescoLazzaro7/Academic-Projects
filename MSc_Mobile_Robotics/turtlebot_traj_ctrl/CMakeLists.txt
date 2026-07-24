cmake_minimum_required(VERSION 3.0.2)
project(turtlebot_traj_ctrl)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp std_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

# 1. Main C++ Node
add_executable(controller_node src/controller_node.cpp)
target_link_libraries(controller_node ${catkin_LIBRARIES})

# This makes the script executable and findable by rosrun
catkin_install_python(PROGRAMS
  scripts/plot_control_results.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)