/* Copyright (c) 2016, Stefan Isler, islerstefan@bluewin.ch
 * (ETH Zurich / Robotics and Perception Group, University of Zurich, Switzerland)
 *
 * This file is part of ig_active_reconstruction, software for information gain based, active reconstruction.
 *
 * ig_active_reconstruction is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 * ig_active_reconstruction is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU Lesser General Public License for more details.
 * Please refer to the GNU Lesser General Public License for details on the license,
 * on <http://www.gnu.org/licenses/>.
*/

#include <stdexcept>
#include <chrono>

#include "ig_active_reconstruction/world_representation_communication_interface.hpp"

#include "ig_active_reconstruction_ros/world_representation_ros_client_ci.hpp"
#include "ig_active_reconstruction_ros/world_conversions.hpp"

#include "ig_active_reconstruction_msgs/srv/information_gain_calculation.hpp"
#include "ig_active_reconstruction_msgs/srv/map_metric_calculation.hpp"
#include "ig_active_reconstruction_msgs/srv/string_list.hpp"


namespace ig_active_reconstruction
{
namespace world_representation
{
  
  RosClientCI::RosClientCI(rclcpp::Node::SharedPtr node)
  : node_(node)
  {
    view_ig_computation_ = node_->create_client<ig_active_reconstruction_msgs::srv::InformationGainCalculation>("world/information_gain");
    map_metric_computation_ = node_->create_client<ig_active_reconstruction_msgs::srv::MapMetricCalculation>("world/map_metric");
    available_ig_receiver_ = node_->create_client<ig_active_reconstruction_msgs::srv::StringList>("world/ig_list");
    available_mm_receiver_ = node_->create_client<ig_active_reconstruction_msgs::srv::StringList>("world/mm_list");
  }
  
  RosClientCI::ResultInformation RosClientCI::computeViewIg(IgRetrievalCommand& command, ViewIgResult& output_ig)
  {
    auto request = std::make_shared<ig_active_reconstruction_msgs::srv::InformationGainCalculation::Request>();
    request->command = ros_conversions::igRetrievalCommandToMsg(command);
    
    while (!view_ig_computation_->wait_for_service()) {
      if (!rclcpp::ok()) {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service: view_ig_computation_. Exiting.");
        return ResultInformation::FAILED;
      }
      RCLCPP_DEBUG(rclcpp::get_logger("rclcpp"), "view_ig_computation_: service not available, waiting again...");
    }

    auto result = view_ig_computation_->async_send_request(request);
    // Wait for the result.
    if (rclcpp::spin_until_future_complete(node_, result) == rclcpp::FutureReturnCode::SUCCESS){
      RCLCPP_DEBUG(rclcpp::get_logger("rclcpp"), "Recieved view_ig_computation");

      for(ig_active_reconstruction_msgs::msg::InformationGain& ig: result.get()->expected_information)
      {
        IgRetrievalResult result = ros_conversions::igRetrievalResultFromMsg(ig);
        output_ig.push_back(result);
      }
      return ResultInformation::SUCCEEDED;
    } else {
      RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to call service add_two_ints");
      
      unsigned int number_of_metrics = (!command.metric_ids.empty())?command.metric_ids.size():command.metric_names.size();
      IgRetrievalResult failed;
      failed.status = ResultInformation::FAILED;
      failed.predicted_gain = 0;
      
      for(unsigned int i=0; i<number_of_metrics; ++i ){
	      output_ig.push_back(failed);
      }
      return ResultInformation::FAILED;
    }
  }

  RosClientCI::ResultInformation RosClientCI::computeMapMetric(MapMetricRetrievalCommand& command, MapMetricRetrievalResultSet& output)
  {
    auto request = std::make_shared<ig_active_reconstruction_msgs::srv::MapMetricCalculation::Request>();
    
    for(const auto& name : command.metric_names)
    {
      request->metric_names.push_back(name);
    }

    while (!map_metric_computation_->wait_for_service()) {
      if (!rclcpp::ok()) {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service: map_metric_computation_. Exiting.");
        return ResultInformation::FAILED;
      }
      RCLCPP_DEBUG(rclcpp::get_logger("rclcpp"), "map_metric_computation_: service not available, waiting again...");
    }
    
    //RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Demanding map metric.");
    auto result = map_metric_computation_->async_send_request(request);    
    // Wait for the result.
    if (rclcpp::spin_until_future_complete(node_, result) == rclcpp::FutureReturnCode::SUCCESS){
      auto response = result.get();
      for(ig_active_reconstruction_msgs::msg::InformationGain& map_metric: response->results)
      {
        MapMetricRetrievalResult result;
        result.status = ros_conversions::resultInformationFromMsg(map_metric.status);
        result.value = map_metric.predicted_gain;
        output.push_back(result);
      }
      return ResultInformation::SUCCEEDED;
    } else {
      MapMetricRetrievalResult failed;
      failed.status = ResultInformation::FAILED;
      failed.value = 0;
      for(unsigned int i=0; i<command.metric_names.size(); ++i)
      {
	      output.push_back(failed);
      }
      return ResultInformation::FAILED;
    }
  }
  
  void RosClientCI::availableIgMetrics(std::vector<MetricInfo>& available_ig_metrics)
  {
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Demanding available information gains.");
    auto request = std::make_shared<ig_active_reconstruction_msgs::srv::StringList::Request>();
    
    while (!available_ig_receiver_->wait_for_service()) {
      if (!rclcpp::ok()) {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service: available_ig_receiver_. Exiting.");
        return;
      }
      RCLCPP_DEBUG(rclcpp::get_logger("rclcpp"), "available_ig_receiver_: service not available, waiting again...");
    }
    
    auto result = available_ig_receiver_->async_send_request(request);
    // Wait for the result.
    if (rclcpp::spin_until_future_complete(node_, result) == rclcpp::FutureReturnCode::SUCCESS){
      auto response = result.get();
      for(unsigned int i=0; i<response->names.size(); ++i)
      {
        MetricInfo new_metric;
        new_metric.name = response->names[i];
        new_metric.id = response->ids[i];
        available_ig_metrics.push_back(new_metric);
      }
    }
  }
  
  void RosClientCI::availableMapMetrics(std::vector<MetricInfo>& available_map_metrics)
  {
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Demanding available map metrics.");
    auto request = std::make_shared<ig_active_reconstruction_msgs::srv::StringList::Request>();
    auto result = available_mm_receiver_->async_send_request(request);
    if (rclcpp::spin_until_future_complete(node_, result) == rclcpp::FutureReturnCode::SUCCESS)
    {
      auto response = result.get();
      for(unsigned int i = 0; i < response->names.size(); ++i)
      {
        MetricInfo new_metric;
        new_metric.name = response->names[i];
        new_metric.id = response->ids[i];
        available_map_metrics.push_back(new_metric);
      }
    }
  }
  
}
}