cmake_minimum_required(VERSION 3.0.2)
project(astar)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  roscpp
)

find_package(OpenCV REQUIRED)

catkin_package()

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(astar_node src/astar_node.cpp)
target_link_libraries(astar_node ${catkin_LIBRARIES} ${OpenCV_LIBS})

