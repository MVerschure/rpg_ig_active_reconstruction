cmake_minimum_required(VERSION 3.5)
project(ig_active_reconstruction_msgs)

set(CMAKE_CXX_STANDARD 17)

find_package(ament_cmake REQUIRED) 
#find_package(cmake_modules REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)

set(msg_files
  "msg/InformationGain.msg"
  "msg/InformationGainRetrievalCommand.msg"
  "msg/InformationGainRetrievalConfig.msg"
  "msg/MovementCostMsg.msg"
  "msg/SubWindow.msg"
  "msg/ViewMsg.msg"
  "msg/ViewSpaceMsg.msg"
)

set(srv_files
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
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  ${srv_files}
  DEPENDENCIES 
  std_msgs
  geometry_msgs
  sensor_msgs
)

#install(
#  DIRECTORY msg
#  DESTINATION share/${PROJECT_NAME}
#)

ament_package()