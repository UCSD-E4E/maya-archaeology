cmake_minimum_required(VERSION 2.8.3)
project(pcl_fuse)

find_package(catkin REQUIRED COMPONENTS
  roscpp std_msgs
  pcl_conversions pcl_ros cv_bridge image_transport
)
include_directories(${catkin_INCLUDE_DIRS})

find_package(PCL REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})

find_package(PkgConfig)
pkg_check_modules(EIGEN3 eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIRS})

find_package(OpenCV)

find_package(OpenMP REQUIRED)
if(OPENMP_FOUND)
  message(STATUS "OPENMP FOUND")
  set(OpenMP_FLAGS ${OpenMP_CXX_FLAGS})
  set(OpenMP_LIBS gomp)
endif()



catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES pcl_fuse
#  CATKIN_DEPENDS other_catkin_pkg
  DEPENDS OpenMP
)



add_executable(pcl_fuse src/pcl_fuse.cpp)
target_compile_options(pcl_fuse PRIVATE ${OpenMP_FLAGS})
add_dependencies(pcl_fuse ${catkin_EXPORTED_TARGETS})
target_link_libraries(pcl_fuse ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBRARIES} ${OpenMP_LIBS})


