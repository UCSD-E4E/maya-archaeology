cmake_minimum_required( VERSION 2.8 )

# Create Project
project( compute_cloud_error )
add_executable( compute_cloud_error compute_cloud_error.cpp )

# Find Packages
find_package( PCL 1.7 REQUIRED )

if( PCL_FOUND )
  include_directories( ${PCL_INCLUDE_DIRS} )

  add_definitions( ${PCL_DEFINITIONS} )
  
  link_directories( ${PCL_LIBRARY_DIRS} )

  target_link_libraries( compute_cloud_error ${PCL_LIBRARIES} )
endif()
