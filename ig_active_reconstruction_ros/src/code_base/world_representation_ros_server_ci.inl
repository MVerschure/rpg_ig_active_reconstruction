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

#define TEMPT template<template<typename> class POINTER_TYPE>
#define CSCOPE RosServerCI<POINTER_TYPE>

#include <stdexcept>
#include <boost/foreach.hpp>

//#include "ig_active_reconstruction_ros/world_representation_ros_server_ci.hpp"
#include "ig_active_reconstruction_ros/world_conversions.hpp"


namespace ig_active_reconstruction
{
namespace world_representation
{
  
template<template<typename> class POINTER_TYPE>
class RosServerCI
{
public:
  RosServerCI(rclcpp::Node::SharedPtr node, POINTER_TYPE<CommunicationInterface> linked_interface)
  : node_(node)
  , linked_interface_(linked_interface)
  {
    view_ig_computation_ = node->create_service<ig_active_reconstruction_msgs::srv::InformationGainCalculation>(
      "world/information_gain", std::bind(&RosServerCI::igComputationService, this, std::placeholders::_1, std::placeholders::_2));
    map_metric_computation_ = node->create_service<ig_active_reconstruction_msgs::srv::MapMetricCalculation>(
      "world/map_metric", std::bind(&RosServerCI::mmComputationService, this, std::placeholders::_1, std::placeholders::_2));
    available_ig_receiver_ = node->create_service<ig_active_reconstruction_msgs::srv::StringList>(
      "world/ig_list", std::bind(&RosServerCI::availableIgService, this, std::placeholders::_1, std::placeholders::_2));
    available_mm_receiver_ = node->create_service<ig_active_reconstruction_msgs::srv::StringList>(
      "world/mm_list", std::bind(&RosServerCI::availableMmService, this, std::placeholders::_1, std::placeholders::_2));
  }
  
  typename ResultInformation computeViewIg(IgRetrievalCommand& command, ViewIgResult& output_ig)
  {
    if (linked_interface_ == nullptr)
      throw std::runtime_error("world_representation::RosServerCI::Interface not linked.");
    
    return linked_interface_->computeViewIg(command, output_ig);
  }
  
  typename ResultInformation computeMapMetric(MapMetricRetrievalCommand& command, MapMetricRetrievalResultSet& output)
  {
    if (linked_interface_ == nullptr)
      throw std::runtime_error("world_representation::RosServerCI::Interface not linked.");
    
    return linked_interface_->computeMapMetric(command, output);
  }
  
  void availableIgMetrics(std::vector<MetricInfo>& available_ig_metrics)
  {
    if (linked_interface_ == nullptr)
      throw std::runtime_error("world_representation::RosServerCI::Interface not linked.");
    
    linked_interface_->availableIgMetrics(available_ig_metrics);
  }
  
  void availableMapMetrics(std::vector<MetricInfo>& available_map_metrics)
  {
    if (linked_interface_ == nullptr)
      throw std::runtime_error("world_representation::RosServerCI::Interface not linked.");
    
    linked_interface_->availableMapMetrics(available_map_metrics);
  }
  
  bool igComputationService(const std::shared_ptr<ig_active_reconstruction_msgs::srv::InformationGainCalculation::Request> request,
    std::shared_ptr<ig_active_reconstruction_msgs::srv::InformationGainCalculation::Response> response)
  {
    RCLCPP_INFO(node_->get_logger("rclcpp"), "Received 'ig computation' call.");
    
    if (linked_interface_ == nullptr)
    {
      ig_active_reconstruction_msgs::InformationGain failed;
      failed.predicted_gain = 0;
      ResultInformation failed_status = ResultInformation::FAILED;
      failed.status = ros_conversions::resultInformationToMsg(failed_status);
      unsigned int number_of_metrics = (!request->command.metric_ids.empty()) ? request->command.metric_ids.size() : request->command.metric_names.size();
      for (unsigned int i = 0; i < number_of_metrics; ++i)
      {
        response->expected_information.push_back(failed);
      }
      return true;
    }
    
    ViewIgResult result;
    IgRetrievalCommand command = ros_conversions::igRetrievalCommandFromMsg(request->command);
    linked_interface_->computeViewIg(command, result);
    
    BOOST_FOREACH(IgRetrievalResult& ig_res, result)
    {
      response->expected_information.push_back(ros_conversions::igRetrievalResultToMsg(ig_res));
    }
    return true;
  }
  
  bool mmComputationService(const std::shared_ptr<ig_active_reconstruction_msgs::srv::MapMetricCalculation::Request> request,
    std::shared_ptr<ig_active_reconstruction_msgs::srv::MapMetricCalculation::Response> response)
  {
    RCLCPP_INFO(node_->get_logger("rclcpp"), "Received 'map metric computation' call.");
    
    if (linked_interface_ == nullptr)
    {
      ig_active_reconstruction_msgs::InformationGain failed;
      failed.predicted_gain = 0;
      ResultInformation failed_status = ResultInformation::FAILED;
      failed.status = ros_conversions::resultInformationToMsg(failed_status);
      unsigned int number_of_metrics = request->metric_names.size();
      for (unsigned int i = 0; i < number_of_metrics; ++i)
      {
        response->results.push_back(failed);
      }
      return true;
    }
    
    MapMetricRetrievalResultSet result;
    MapMetricRetrievalCommand command;
    BOOST_FOREACH(std::string& name, request->metric_names)
    {
      command.metric_names.push_back(name);
    }
    linked_interface_->computeMapMetric(command, result);
    
    BOOST_FOREACH(MapMetricRetrievalResult& ig_res, result)
    {
      ig_active_reconstruction_msgs::InformationGain gain;
      gain.predicted_gain = ig_res.value;
      gain.status = ros_conversions::resultInformationToMsg(ig_res.status);
      response->results.push_back(gain);
    }
    return true;
  }
  
  bool availableIgService(const std::shared_ptr<ig_active_reconstruction_msgs::srv::StringList::Request> request,
    std::shared_ptr<ig_active_reconstruction_msgs::srv::StringList::Response> response)
  {
    RCLCPP_INFO(node_->get_logger("rclcpp"), "Received 'available information gain metric' call.");
    
    if (linked_interface_ == nullptr)
    {
      return true;
    }
    
    std::vector<MetricInfo> metric_list;
    linked_interface_->availableIgMetrics(metric_list);
    
    BOOST_FOREACH(MetricInfo& metric, metric_list)
    {
      response->names.push_back(metric.name);
      response->ids.push_back(metric.id);
    }
    return true;
  }
  
  bool availableMmService(const std::shared_ptr<ig_active_reconstruction_msgs::srv::StringList::Request> request,
    std::shared_ptr<ig_active_reconstruction_msgs::srv::StringList::Response> response)
  {
    RCLCPP_INFO(node_->get_logger("rclcpp"), "Received 'available map metric' call.");
    
    if (linked_interface_ == nullptr)
    {
      return true;
    }
    
    std::vector<MetricInfo> metric_list;
    linked_interface_->availableMapMetrics(metric_list);
    
    BOOST_FOREACH(MetricInfo& metric, metric_list)
    {
      response->names.push_back(metric.name);
      response->ids.push_back(metric.id);
    }
    return true;
  }
  
}

}

#undef CSCOPE
#undef TEMPT