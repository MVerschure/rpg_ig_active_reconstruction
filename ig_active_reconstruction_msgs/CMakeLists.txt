cmake_minimum_required(VERSION 3.5)
project(ig_active_reconstruction_msgs)

find_package(ament_cmake REQUIRED) 
#find_package(cmake_modules REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/InformationGain.msg"
  "msg/InformationGainRetrievalCommand.msg"
  "msg/InformationGainRetrievalConfig.msg"
  "msg/MovementCostMsg.msg"
  "msg/SubWindow.msg"
  "msg/ViewMsg.msg"
  "msg/ViewSpaceMsg.msg"
  "srv/DeleteViews.srv"
  "srv/InformationGainCalculation.srv"
  "srv/MapMetricCalculation.srv"
  "srv/MovementCostCalculation.srv"
  "srv/MoveToOrder.srv"
  "srv/PclInput.srv"
  "srv/RetrieveData.srv"
  "srv/StringList.srv"
  "srv/ViewRequest.srv"
  "srv/ViewSpaceRequest.srv"
  "srv/ViewSpaceUpdate.srv"
  DEPENDENCIES 
  std_msgs
  geometry_msgs
  sensor_msgs
)

ament_export_dependencies(message_runtime)

include_directories(include
  ${catkin_INCLUDE_DIRS}
)

ament_package()