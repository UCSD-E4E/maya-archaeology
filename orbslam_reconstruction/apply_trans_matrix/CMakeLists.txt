cmake_minimum_required(VERSION 2.6 FATAL_ERROR)
set (CMAKE_CXX_STANDARD 11)

project(pcl-matrix_transform)

find_package(PCL 1.7 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable (matrix_transform src/matrix_transform.cpp)
target_link_libraries (matrix_transform ${PCL_LIBRARIES})
