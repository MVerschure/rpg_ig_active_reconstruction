cmake_minimum_required(VERSION 3.5)
project(ig_active_reconstruction_octomap)

## c++11 is preferred but ROS is built with c++03 and the PCL binaries are not compatible (boost)
##list( APPEND CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")

set(CMAKE_CXX_STANDARD 17)

find_package(ament_cmake REQUIRED)
find_package(ig_active_reconstruction REQUIRED)
find_package(rclcpp REQUIRED)
find_package(ig_active_reconstruction_ros REQUIRED)
find_package(movements REQUIRED)
find_package(ig_active_reconstruction_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(octomap REQUIRED)
find_package(Boost REQUIRED)
find_package(PCL 1.7 REQUIRED)
find_package(Eigen REQUIRED)


include_directories(include
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${Eigen_INCLUDE_DIRS}
  ${OCTOMAP_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
)

file(GLOB ${PROJECT_NAME}_CODE_BASE
  "src/code_base/*.cpp"
  "src/code_base/ig/*.cpp"
)

set(${PROJECT_NAME}_LIBRARIES
   ${PCL_LIBRARIES}
   ${OCTOMAP_LIBRARIES}
   ${Boost_LIBRARIES}
)


add_library(${PROJECT_NAME} STATIC
  ${${PROJECT_NAME}_CODE_BASE}
)

target_link_libraries(${PROJECT_NAME}
   ${${PROJECT_NAME}_LIBRARIES}
)
ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  ig_active_reconstruction_ros
  ig_active_reconstruction_msgs
  ig_active_reconstruction
  movements
  tf2
  tf2_ros
  sensor_msgs
  pcl_ros
  visualization_msgs
)

# Executables...........................................................
add_executable(octomap_world_representation
  src/ros_nodes/octomap_world_representation.cpp
  ${${PROJECT_NAME}_CODE_BASE}
)
target_link_libraries(octomap_world_representation
   ${${PROJECT_NAME}_LIBRARIES}
)
ament_target_dependencies(octomap_world_representation
  rclcpp
  ig_active_reconstruction_ros
  ig_active_reconstruction_msgs
  ig_active_reconstruction
  movements
  tf2
  tf2_ros
  sensor_msgs
  pcl_ros
  visualization_msgs
)

# Launch files...........................................................
install(
  DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}
)

# Ament package..........................................................
ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(
  movements
  ig_active_reconstruction
  rclcpp
  ig_active_reconstruction_ros
  ig_active_reconstruction_msgs
  sensor_msgs
  tf2
  pcl_ros
  visualization_msgs
  geometry_msgs
  std_msgs
)
ament_package()