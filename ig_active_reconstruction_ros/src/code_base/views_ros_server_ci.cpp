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

#include "ig_active_reconstruction/views_communication_interface.hpp"
#include "ig_active_reconstruction_ros/views_ros_server_ci.hpp"
#include "ig_active_reconstruction_ros/views_conversions.hpp"

namespace ig_active_reconstruction
{
namespace views
{
  RosServerCI::RosServerCI(rclcpp::Node::SharedPtr node, std::shared_ptr<CommunicationInterface> linked_interface)
  : node_(node)
  , linked_interface_(linked_interface)
  {
    viewspace_service_ = node_->create_service<ig_active_reconstruction_msgs::srv::ViewSpaceRequest>("views/space",
      std::bind(&RosServerCI::viewspaceService, this, std::placeholders::_1, std::placeholders::_2));
    
    views_adder_service_ = node_->create_service<ig_active_reconstruction_msgs::srv::ViewSpaceUpdate>("views/add",
      std::bind(&RosServerCI::viewsAdderService, this, std::placeholders::_1, std::placeholders::_2));
    
    views_deleter_service_ = node_->create_service<ig_active_reconstruction_msgs::srv::DeleteViews>("views/delete",
      std::bind(&RosServerCI::viewsDeleterService, this, std::placeholders::_1, std::placeholders::_2));
  }

  const ViewSpace& RosServerCI::getViewSpace()
  {
    if (linked_interface_ == nullptr)
      throw std::runtime_error("views::RosServerCI::Interface not linked.");

    return linked_interface_->getViewSpace();
  }

  RosServerCI::ViewSpaceUpdateResult RosServerCI::addViews(std::vector<View>& new_views)
  {
    if (linked_interface_ == nullptr)
      throw std::runtime_error("views::RosServerCI::Interface not linked.");

    return linked_interface_->addViews(new_views);
  }

  RosServerCI::ViewSpaceUpdateResult RosServerCI::addView(View new_view)
  {
    if (linked_interface_ == nullptr)
      throw std::runtime_error("views::RosServerCI::Interface not linked.");

    return linked_interface_->addView(new_view);
  }

  RosServerCI::ViewSpaceUpdateResult RosServerCI::deleteViews(std::vector<View::IdType>& view_ids)
  {
    if (linked_interface_ == nullptr)
      throw std::runtime_error("views::RosServerCI::Interface not linked.");

    return linked_interface_->deleteViews(view_ids);
  }

  RosServerCI::ViewSpaceUpdateResult RosServerCI::deleteView(View::IdType view_id)
  {
    if (linked_interface_ == nullptr)
      throw std::runtime_error("views::RosServerCI::Interface not linked.");

    return linked_interface_->deleteView(view_id);
  }

  bool RosServerCI::viewspaceService(
    const std::shared_ptr<ig_active_reconstruction_msgs::srv::ViewSpaceRequest::Request> request,
    std::shared_ptr<ig_active_reconstruction_msgs::srv::ViewSpaceRequest::Response> response)
  {
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Received 'viewspace' call.");
    if (linked_interface_ == nullptr)
    {
      ViewSpaceStatus status = ViewSpaceStatus::BAD;
      response->viewspace_status = ros_conversions::viewSpaceStatusToMsg(status);
      return true;
    }

    const ViewSpace& viewspace = linked_interface_->getViewSpace();
    response->viewspace = ros_conversions::viewSpaceToMsg(viewspace);
    ViewSpaceStatus status = ViewSpaceStatus::OK;
    response->viewspace_status = ros_conversions::viewSpaceStatusToMsg(status);
    return true;
  }

  bool RosServerCI::viewsAdderService(
    const std::shared_ptr<ig_active_reconstruction_msgs::srv::ViewSpaceUpdate::Request> request,
    std::shared_ptr<ig_active_reconstruction_msgs::srv::ViewSpaceUpdate::Response> response)
  {
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Received 'add view(s)' call.");
    if (linked_interface_ == nullptr)
    {
      RosServerCI::ViewSpaceUpdateResult result = RosServerCI::ViewSpaceUpdateResult::NOT_AVAILABLE;
      response->update_result = ros_conversions::viewSpaceUpdateResultToMsg(result);
      return false;
    }

    std::vector<View> new_views;
    for (auto& view_msg : request->views)
    {
      new_views.push_back(ros_conversions::viewFromMsg(view_msg));
    }
    RosServerCI::ViewSpaceUpdateResult result = linked_interface_->addViews(new_views);

    response->update_result = ros_conversions::viewSpaceUpdateResultToMsg(result);
    return true;
  }

  bool RosServerCI::viewsDeleterService(
    const std::shared_ptr<ig_active_reconstruction_msgs::srv::DeleteViews::Request> request,
    std::shared_ptr<ig_active_reconstruction_msgs::srv::DeleteViews::Response> response)
  {
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Received 'delete view(s)' call.");
    if (linked_interface_ == nullptr)
    {
      RosServerCI::ViewSpaceUpdateResult result = RosServerCI::ViewSpaceUpdateResult::NOT_AVAILABLE;
      
      response->update_result = ros_conversions::viewSpaceUpdateResultToMsg(result);
      return false;
    }

    std::vector<View::IdType> delete_ids;
    for (auto& id : request->ids)
    {
      delete_ids.push_back(id);
    }
    RosServerCI::ViewSpaceUpdateResult result = linked_interface_->deleteViews(delete_ids);
    response->update_result = ros_conversions::viewSpaceUpdateResultToMsg(result);
    return true;
  }
}
}