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

#include "ig_active_reconstruction_ros/robot_conversions.hpp"
#include "ig_active_reconstruction_ros/views_conversions.hpp"
#include "ig_active_reconstruction_ros/robot_ros_client_ci.hpp"

namespace ig_active_reconstruction
{
namespace robot
{
  
  RosClientCI::RosClientCI( rclcpp::Node::SharedPtr node )
  : node_(node)
  {
    current_view_retriever_ = node_->create_client<ig_active_reconstruction_msgs::srv::ViewRequest>("robot/current_view");
    data_retriever_ = node_->create_client<ig_active_reconstruction_msgs::srv::RetrieveData>("robot/retrieve_data");
    cost_retriever_ = node_->create_client<ig_active_reconstruction_msgs::srv::MovementCostCalculation>("robot/movement_cost");
    robot_mover_ = node_->create_client<ig_active_reconstruction_msgs::srv::MoveToOrder>("robot/move_to");
  }
  
  views::View RosClientCI::getCurrentView()
  {
    auto request = std::make_shared<ig_active_reconstruction_msgs::srv::ViewRequest::Request>();

    while (!current_view_retriever_->wait_for_service()) {
      if (!rclcpp::ok()) {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service current_view_retriever_. Exiting.");
        throw std::runtime_error("shit");
      }
      RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "current_view_retriever_: service not available, waiting again...");
    }
      
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Demanding current view");

    auto result = current_view_retriever_->async_send_request(request);
    
    if (rclcpp::spin_until_future_complete(node_, result) == rclcpp::FutureReturnCode::SUCCESS)
    {
      return ros_conversions::viewFromMsg(result.get()->view);
    } else {
      throw std::runtime_error("RosClientCI::getCurrentView failed for unknown reason.");
    }
  }
  
  RosClientCI::ReceptionInfo RosClientCI::retrieveData()
  {
    auto request = std::make_shared<ig_active_reconstruction_msgs::srv::RetrieveData::Request>();
    
    while (!data_retriever_->wait_for_service()) {
      if (!rclcpp::ok()) {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service data_retriever_. Exiting.");
        throw std::runtime_error("shit");
      }
      RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "data_retriever_: service not available, waiting again...");
    }

    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Retrieving data");
    auto result = data_retriever_->async_send_request(request);
    if (rclcpp::spin_until_future_complete(node_, result) != rclcpp::FutureReturnCode::SUCCESS)
    {
      throw std::runtime_error("RosClientCI::retrieveData failed for unknown reason.");
    }
    
    return ros_conversions::robotReceptionInfoFromMsg(result.get()->receive_info);
  }
  
  MovementCost RosClientCI::movementCost( views::View& target_view )
  {
    auto request = std::make_shared<ig_active_reconstruction_msgs::srv::MovementCostCalculation::Request>();
    views::View current_view = getCurrentView();
    
    request->start_view = ros_conversions::viewToMsg(current_view);
    request->target_view = ros_conversions::viewToMsg(target_view);
    request->additional_information = true;
    
    while (!cost_retriever_->wait_for_service()) {
      if (!rclcpp::ok()) {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service cost_retriever_. Exiting.");
        throw std::runtime_error("shit");
      }
      RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "cost_retriever_: service not available, waiting again...");
    }

    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Retrieving movement cost");
    auto result = cost_retriever_->async_send_request(request);
    if (rclcpp::spin_until_future_complete(node_, result) != rclcpp::FutureReturnCode::SUCCESS)
    {
      MovementCost cost;
      cost.exception = MovementCost::Exception::RECEPTION_FAILED;
      return cost;
    }
    
    return ros_conversions::movementCostFromMsg(result.get()->movement_cost);
  }
  
  MovementCost RosClientCI::movementCost( views::View& start_view, views::View& target_view, bool fill_additional_information )
  {
    auto request = std::make_shared<ig_active_reconstruction_msgs::srv::MovementCostCalculation::Request>();
    
    request->start_view = ros_conversions::viewToMsg(start_view);
    request->target_view = ros_conversions::viewToMsg(target_view);
    request->additional_information = fill_additional_information;
    
    while (!cost_retriever_->wait_for_service()) {
      if (!rclcpp::ok()) {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service cost_retriever_. Exiting.");
        throw std::runtime_error("shit");
      }
      RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "cost_retriever_: service not available, waiting again...");
    }

    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Retrieving movement cost");
    auto result = cost_retriever_->async_send_request(request);
    if (rclcpp::spin_until_future_complete(node_, result) != rclcpp::FutureReturnCode::SUCCESS)
    {
      MovementCost cost;
      cost.exception = MovementCost::Exception::RECEPTION_FAILED;
      return cost;
    }
    
    return ros_conversions::movementCostFromMsg(result.get()->movement_cost);
  }
  
  bool RosClientCI::moveTo( views::View& target_view )
  {
    auto request = std::make_shared<ig_active_reconstruction_msgs::srv::MoveToOrder::Request>();
    request->target_view = ros_conversions::viewToMsg(target_view);
    
    while (!robot_mover_->wait_for_service()) {
      if (!rclcpp::ok()) {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service robot_mover_. Exiting.");
        throw std::runtime_error("shit");
      }
      RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "robot_mover_: service not available, waiting again...");
    }

    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Demanding robot to move.");
    auto result = robot_mover_->async_send_request(request);
    if (rclcpp::spin_until_future_complete(node_, result) != rclcpp::FutureReturnCode::SUCCESS)
    {
      return false;
    }
    
    return result.get()->success;
  }
}
}