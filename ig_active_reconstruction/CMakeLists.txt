cmake_minimum_required(VERSION 3.5)
project(ig_active_reconstruction)


# Default to C++17
set(CMAKE_CXX_STANDARD 17)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(movements REQUIRED)
find_package(Boost REQUIRED)


include_directories(include
  #${catkin_INCLUDE_DIRS}
  ${movements_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
)

INCLUDE_DIRECTORIES ( "$ENV{EIGEN3_INCLUDE_DIR}" )

file(GLOB ${PROJECT_NAME}_CODE_BASE
  "src/code_base/*.cpp"
  "src/code_base/ig/*.cpp"
)

add_library(${PROJECT_NAME} STATIC
  ${${PROJECT_NAME}_CODE_BASE}
)

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(movements)
target_link_libraries(${PROJECT_NAME}
   ${catkin_LIBRARIES}
   ${Boost_LIBRARIES}
)

install(
  DIRECTORY include/
  DESTINATION include
)


install(
    TARGETS ${PROJECT_NAME}
    EXPORT export_${PROJECT_NAME}
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib
    RUNTIME DESTINATION bin
    INCLUDES DESTINATION include
)

ament_export_include_directories(include)
#ament_export_libraries(${PROJECT_NAME})

ament_package()