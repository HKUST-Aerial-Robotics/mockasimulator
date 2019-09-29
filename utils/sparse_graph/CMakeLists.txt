cmake_minimum_required(VERSION 2.8.3)
project(sparse_graph)

add_definitions(-std=c++11)

find_package(catkin REQUIRED
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp
)

include_directories(
    ${catkin_INCLUDE_DIRS}
    include
    )
