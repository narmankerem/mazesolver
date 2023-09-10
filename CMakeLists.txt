cmake_minimum_required(VERSION 3.0.2)
project(mazesolver)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  roscpp
  rospy
  rosserial_arduino
  rosserial_python
  sensor_msgs
  std_msgs
  tf
)

catkin_package(
  CATKIN_DEPENDS geometry_msgs nav_msgs roscpp rospy rosserial_arduino rosserial_python sensor_msgs std_msgs tf
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)



add_executable(maze_solver_node src/maze_solver.cpp)
add_dependencies(maze_solver_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(maze_solver_node ${catkin_LIBRARIES})

