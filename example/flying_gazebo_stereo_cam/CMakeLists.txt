cmake_minimum_required(VERSION 2.8.3)
project(flying_gazebo_stereo_cam)
set(CMAKE_BUILD_TYPE Release)

list( APPEND CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")

find_package(ament_cmake REQUIRED)
find_package(cmake_modules REQUIRED)
find_package(roscpp REQUIRED)
find_package(urdf REQUIRED)
find_package(ig_active_reconstruction REQUIRED)
find_package(ig_active_reconstruction_msgs REQUIRED)
find_package(ig_active_reconstruction_ros REQUIRED)
find_package(ig_active_reconstruction_octomap REQUIRED)
find_package(movements REQUIRED)
find_package(tf REQUIRED)
find_package(gazebo_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

find_package(Eigen REQUIRED)
find_package(Boost REQUIRED)

ament_export_dependencies(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(
  roscpp
  urdf
  ig_active_reconstruction
  ig_active_reconstruction_msgs
  ig_active_reconstruction_ros
  ig_active_reconstruction_octomap
  movements
  geometry_msgs
  sensor_msgs
)

include_directories(include
  ${catkin_INCLUDE_DIRS}
  ${Eigen_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
)

file(GLOB ${PROJECT_NAME}_CODE_BASE
  "src/code_base/*.cpp"
)

add_library(${PROJECT_NAME} STATIC
  ${${PROJECT_NAME}_CODE_BASE}
)

target_link_libraries(${PROJECT_NAME}
   ${catkin_LIBRARIES}
   ${Boost_LIBRARIES}
)

# Executables...........................................................

add_executable(robot_interface
  src/ros_nodes/robot_interface.cpp
  ${${PROJECT_NAME}_CODE_BASE}
)
target_link_libraries(robot_interface
   ${catkin_LIBRARIES}
   ${Boost_LIBRARIES}
)
add_dependencies(robot_interface
 ${catkin_EXPORTED_TARGETS}
)


ament_package()