cmake_minimum_required(VERSION 3.5)
project(ig_active_reconstruction_ros)

set(CMAKE_CXX_STANDARD 17)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(ig_active_reconstruction_msgs REQUIRED)
find_package(ig_active_reconstruction REQUIRED)
find_package(movements REQUIRED)


file(GLOB ${PROJECT_NAME}_CODE_BASE
  "src/code_base/*.cpp"
)


add_library(${PROJECT_NAME} STATIC
  ${${PROJECT_NAME}_CODE_BASE}
)


# Executables...........................................................

add_executable(basic_view_planner
  src/ros_nodes/basic_view_planner.cpp
  ${${PROJECT_NAME}_CODE_BASE}
)
ament_target_dependencies(basic_view_planner
  rclcpp
)

add_executable(simple_viewspace_module
  src/ros_nodes/simple_viewspace_module.cpp
  ${${PROJECT_NAME}_CODE_BASE}
)
ament_target_dependencies(simple_viewspace_module
 rclcpp
)

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(
  movements
  roscpp
  ig_active_reconstruction_msgs
  ig_active_reconstruction
)
ament_package()
