cmake_minimum_required(VERSION 3.0.2)
project(fast_lio2_loop)

# ---- C++ standard (model-style: explicit & required)
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# ---- Catkin & ROS deps
find_package(catkin REQUIRED COMPONENTS
  roscpp
  nav_msgs
  sensor_msgs
  std_msgs
  pcl_ros
  tf2_ros
  tf2_geometry_msgs
)

# ---- System deps (add or remove as needed)
find_package(Eigen3 REQUIRED)

# If you directly use PCL APIs (not only pcl_ros), uncomment this:
# find_package(PCL REQUIRED COMPONENTS common io filters)

# ---- Catkin package export
# If you have headers in include/ and/or you build a library, expose them here.
catkin_package(
  INCLUDE_DIRS include
  # LIBRARIES fast_lio2_loop
  CATKIN_DEPENDS roscpp nav_msgs sensor_msgs std_msgs pcl_ros tf2_ros tf2_geometry_msgs
  # DEPENDS EIGEN3
)

# ---- Include paths
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  # ${PCL_INCLUDE_DIRS}   # uncomment if find_package(PCL) is used
)

# ---- Optional: link dirs for PCL if used
# link_directories(${PCL_LIBRARY_DIRS})

add_executable(fastlio_keyframe_tap

              src/fast_lio_sub.cpp
              src/keyframeCapture.cpp
)

target_link_libraries(fastlio_keyframe_tap ${catkin_LIBRARIES})

add_dependencies(fastlio_keyframe_tap 
                ${${PROJECT_NAME}_EXPORTED_TARGETS} 
                ${catkin_EXPORTED_TARGETS}
                
)

