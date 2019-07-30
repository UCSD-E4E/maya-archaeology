cmake_minimum_required(VERSION 2.8.3)
project(sync_example)

find_package(catkin REQUIRED COMPONENTS message_filters roscpp sensor_msgs)

catkin_package(
  CATKIN_DEPENDS message_filters roscpp sensor_msgs
)

include_directories(${catkin_INCLUDE_DIRS})

add_executable(sync_example sync_example.cpp)
target_link_libraries(sync_example ${catkin_LIBRARIES})
