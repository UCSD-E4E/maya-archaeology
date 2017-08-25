cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(pcd_goicp)

find_package(PCL 1.2 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable (pcd_normalize pcd_normalize.cpp)
target_link_libraries (pcd_normalize ${PCL_LIBRARIES})

add_executable (pcd_transform pcd_transform.cpp)
target_link_libraries (pcd_transform ${PCL_LIBRARIES})
