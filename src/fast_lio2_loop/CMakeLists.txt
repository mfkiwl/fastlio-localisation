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
  # INCLUDE_DIRS include
  # LIBRARIES fast_lio2_loop
  # CATKIN_DEPENDS roscpp nav_msgs sensor_msgs std_msgs pcl_ros tf2_ros tf2_geometry_msgs
  # DEPENDS EIGEN3
)

# ---- Include paths
include_directories(
  # include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  # ${PCL_INCLUDE_DIRS}   # uncomment if find_package(PCL) is used
)

# ---- Optional: link dirs for PCL if used
# link_directories(${PCL_LIBRARY_DIRS})

add_executable(fastlio_keyframe_tap src/fastlio_keyframe_tap.cpp)
target_link_libraries(fastlio_keyframe_tap ${catkin_LIBRARIES})
add_dependencies(fastlio_keyframe_tap ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

# ---- Helper function to add ROS executables consistently
function(add_ros_exe target)
  # Usage: add_ros_exe(name src/file.cpp [src/other.cpp ...])
  add_executable(${ARGV})
  add_dependencies(${target} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
  target_link_libraries(${target}
    ${catkin_LIBRARIES}
    # ${PCL_LIBRARIES}  # uncomment if you linked PCL above
  )
  # Keep target on C++14 even if toolchain defaults differ
  target_compile_features(${target} PUBLIC cxx_std_14)
endfunction()

# ---- Add your executables here (examples)
# Replace/duplicate these lines for each node you have
# add_ros_exe(flio_loop_node src/fast_lio2_loop_node.cpp)
# add_ros_exe(odom_to_tf_node src/odom_to_tf_node.cpp)
# add_ros_exe(map_saver_node src/map_saver_node.cpp src/utils/map_io.cpp)

# ---- Install (so binaries appear in devel/install spaces)
# install(TARGETS
#   flio_loop_node
#   odom_to_tf_node
#   map_saver_node
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

# ---- (Optional) Python scripts
# catkin_install_python(PROGRAMS
#   scripts/some_helper.py
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

# ---- (Optional) Headers and shared data
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h" PATTERN "*.hpp"
# )
# install(DIRECTORY launch cfg params
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
#   OPTIONAL
# )
