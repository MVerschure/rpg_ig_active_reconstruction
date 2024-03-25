cmake_minimum_required(VERSION 3.5)
project(ig_active_reconstruction_ros)

set(CMAKE_CXX_STANDARD 17)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(ig_active_reconstruction_msgs REQUIRED)
find_package(ig_active_reconstruction REQUIRED)
find_package(movements REQUIRED)
find_package(Boost REQUIRED COMPONENTS thread chrono)
find_package(Eigen3 REQUIRED)

include_directories(include
  ${EIGEN3_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
#  ig_active_reconstruction_msgs_INCLUDE_DIRS
)

# Library -----------------------------------------------

file(GLOB ${PROJECT_NAME}_CODE_BASE
  "src/code_base/*.cpp"
)

add_library(${PROJECT_NAME}
  ${${PROJECT_NAME}_CODE_BASE}
)

ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  ig_active_reconstruction_msgs
  ig_active_reconstruction
  movements
)

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)


install(
    TARGETS ${PROJECT_NAME}
    EXPORT export_${PROJECT_NAME}
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib
    RUNTIME DESTINATION bin
    INCLUDES DESTINATION include
)


# Executables...........................................................

add_executable(basic_view_planner
  src/ros_nodes/basic_view_planner.cpp
  ${${PROJECT_NAME}_CODE_BASE}
)
ament_target_dependencies(basic_view_planner
  rclcpp
  ig_active_reconstruction_msgs
  ig_active_reconstruction
  movements
)
target_link_libraries(basic_view_planner
  ${Boost_LIBRARIES}
)


add_executable(simple_viewspace_module
  src/ros_nodes/simple_viewspace_module.cpp
  ${${PROJECT_NAME}_CODE_BASE}
)
ament_target_dependencies(simple_viewspace_module
  rclcpp
  ig_active_reconstruction_msgs
  ig_active_reconstruction
  movements
)

# Include -------------------------------------------------

install(
  DIRECTORY include/
  DESTINATION include
)

# Launch files --------------------------------------------
install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)


ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(
  movements
  rclcpp
  ig_active_reconstruction_msgs
  ig_active_reconstruction
)
ament_package()
