cmake_minimum_required(VERSION 3.0.2)
project(turtlebot_simulator)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  rosgraph_msgs
)

find_package(Boost REQUIRED)

catkin_package(
  CATKIN_DEPENDS roscpp std_msgs rosgraph_msgs
  INCLUDE_DIRS include
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
)

# Integrator Library
add_library(ode_integrator src/ode_integrator.cpp)

# Robot Simulator Node
add_executable(robot_simulator src/robot_simulator_node.cpp)
target_link_libraries(robot_simulator ode_integrator ${catkin_LIBRARIES})

# Test Node
add_executable(test_node src/test_node.cpp)
target_link_libraries(test_node ${catkin_LIBRARIES})

# Install scripts (Python)
catkin_install_python(PROGRAMS
  scripts/plot_test.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)