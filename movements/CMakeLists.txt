cmake_minimum_required(VERSION 3.5)
project(movements)

set(CMAKE_CXX_STANDARD 17)

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(angles REQUIRED)


include_directories(include
  #${Eigen_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
)

INCLUDE_DIRECTORIES ( "$ENV{EIGEN3_INCLUDE_DIR}" )

file(GLOB ${PROJECT_NAME}_CODE_BASE
  "src/*.cpp"
)

add_library(${PROJECT_NAME}
  ${${PROJECT_NAME}_CODE_BASE}
)
ament_target_dependencies(${PROJECT_NAME}
  geometry_msgs
  angles
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

install(
  DIRECTORY include/
  DESTINATION include
)

ament_export_dependencies(angles)
ament_export_include_directories(include)
#ament_export_libraries(${PROJECT_NAME})

ament_package()