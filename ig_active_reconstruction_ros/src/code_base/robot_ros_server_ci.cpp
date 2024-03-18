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
#include "ig_active_reconstruction_ros/robot_ros_server_ci.hpp"
#include "ig_active_reconstruction_ros/robot_conversions.hpp"
#include "ig_active_reconstruction_ros/views_conversions.hpp"

namespace ig_active_reconstruction
{
namespace robot
{
  
  RosServerCI::RosServerCI( rclcpp::Node::SharedPtr node, std::shared_ptr<CommunicationInterface> linked_interface )
  : node_(node)
  , linked_interface_(linked_interface)
  {
    current_view_service_ = node_->create_service<ig_active_reconstruction_msgs::srv::ViewRequest>("robot/current_view",
      std::bind(&RosServerCI::currentViewService, this, std::placeholders::_1, std::placeholders::_2));
    data_service_ = node_->create_service<ig_active_reconstruction_msgs::srv::RetrieveData>("robot/retrieve_data",
      std::bind(&RosServerCI::retrieveDataService, this, std::placeholders::_1, std::placeholders::_2));
    cost_service_ = node_->create_service<ig_active_reconstruction_msgs::srv::MovementCostCalculation>("robot/movement_cost",
      std::bind(&RosServerCI::movementCostService, this, std::placeholders::_1, std::placeholders::_2));
    robot_moving_service_ = node_->create_service<ig_active_reconstruction_msgs::srv::MoveToOrder>("robot/move_to",
      std::bind(&RosServerCI::moveToService, this, std::placeholders::_1, std::placeholders::_2));
  }
  
  views::View RosServerCI::getCurrentView()
  {
    if( linked_interface_ == nullptr )
      throw std::runtime_error("robot::RosServerCI::Interface not linked.");
    
    return linked_interface_->getCurrentView();
  }
  
  RosServerCI::ReceptionInfo RosServerCI::retrieveData()
  {
    if( linked_interface_ == nullptr )
      throw std::runtime_error("robot::RosServerCI::Interface not linked.");
    
    return linked_interface_->retrieveData();
  }
  
  MovementCost RosServerCI::movementCost( views::View& target_view )
  {
    if( linked_interface_ == nullptr )
      throw std::runtime_error("robot::RosServerCI::Interface not linked.");
    
    return linked_interface_->movementCost(target_view);
  }
  
  MovementCost RosServerCI::movementCost( views::View& start_view, views::View& target_view, bool fill_additional_information )
  {
    if( linked_interface_ == nullptr )
      throw std::runtime_error("robot::RosServerCI::Interface not linked.");
    
    return linked_interface_->movementCost(start_view, target_view, fill_additional_information);
  }
  
  bool RosServerCI::moveTo( views::View& target_view )
  {
    if( linked_interface_ == nullptr )
      throw std::runtime_error("robot::RosServerCI::Interface not linked.");
    
    return linked_interface_->moveTo(target_view);
  }
  
  bool RosServerCI::currentViewService(const std::shared_ptr<ig_active_reconstruction_msgs::srv::ViewRequest::Request> request,
    std::shared_ptr<ig_active_reconstruction_msgs::srv::ViewRequest::Response> response)
  {
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Received current view call.");
    
    if( linked_interface_ == nullptr )
    {
      return false;
    }
    
    views::View current_view = linked_interface_->getCurrentView();
    response->view = ros_conversions::viewToMsg(current_view);
    
    return true;
  }
  
  bool RosServerCI::retrieveDataService(const std::shared_ptr<ig_active_reconstruction_msgs::srv::RetrieveData::Request> request,
    std::shared_ptr<ig_active_reconstruction_msgs::srv::RetrieveData::Response> response)
  {
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Received 'retrieve data' call.");
    
    if( linked_interface_ == nullptr )
    {
      ReceptionInfo inf = ReceptionInfo::FAILED;
      response->receive_info = ros_conversions::robotReceptionInfoToMsg(inf);
      return true;
    }
    
    ReceptionInfo rec_info = linked_interface_->retrieveData();
    
    response->receive_info = ros_conversions::robotReceptionInfoToMsg(rec_info);
    return true;
  }
  
  bool RosServerCI::movementCostService(const std::shared_ptr<ig_active_reconstruction_msgs::srv::MovementCostCalculation::Request> request,
    std::shared_ptr<ig_active_reconstruction_msgs::srv::MovementCostCalculation::Response> response)
  {
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Received 'movement cost' call.");
    MovementCost cost;
    
    if( linked_interface_ == nullptr )
    {
      cost.exception = MovementCost::Exception::RECEPTION_FAILED;
    }
    else
    {
      views::View start_view = ros_conversions::viewFromMsg(request->start_view);
      views::View target_view = ros_conversions::viewFromMsg(request->target_view);
      bool fill_additional_info = request->additional_information;
      
      cost = linked_interface_->movementCost(start_view, target_view, fill_additional_info);
    }
    
    response->movement_cost = ros_conversions::movementCostToMsg(cost);
    return true;
  }
  
  bool RosServerCI::moveToService(const std::shared_ptr<ig_active_reconstruction_msgs::srv::MoveToOrder::Request> request,
    std::shared_ptr<ig_active_reconstruction_msgs::srv::MoveToOrder::Response> response)
  {
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Received 'move to position' call.");
    
    if( linked_interface_ == nullptr )
    {
      response->success = false;
      return true;
    }
    
    views::View target_view = ros_conversions::viewFromMsg(request->target_view);
    
    bool success = linked_interface_->moveTo(target_view);
    
    response->success = success;
    return true;
  }
}
}