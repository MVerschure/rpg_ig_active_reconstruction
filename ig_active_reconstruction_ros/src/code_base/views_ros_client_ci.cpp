#include <stdexcept>

#include "ig_active_reconstruction_ros/views_conversions.hpp"
#include "ig_active_reconstruction_ros/views_ros_client_ci.hpp"

#include "ig_active_reconstruction_msgs/srv/retrieve_data.hpp"
#include "ig_active_reconstruction_msgs/srv/movement_cost_calculation.hpp"
#include "ig_active_reconstruction_msgs/srv/move_to_order.hpp"

namespace ig_active_reconstruction
{
namespace views
{
  
  RosClientCI::RosClientCI( rclcpp::Node::SharedPtr node )
  : node_(node)
  {
    planning_space_receiver_ = node_->create_client<ig_active_reconstruction_msgs::srv::ViewSpaceRequest>("views/space");
    views_adder_ = node_->create_client<ig_active_reconstruction_msgs::srv::ViewSpaceUpdate>("views/add");
    views_deleter_ = node_->create_client<ig_active_reconstruction_msgs::srv::DeleteViews>("views/delete");
  }
  
  const ViewSpace& RosClientCI::getViewSpace()
  {
    auto request = std::make_shared<ig_active_reconstruction_msgs::srv::ViewSpaceRequest::Request>();
    
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Demanding viewspace.");
    auto result = planning_space_receiver_->async_send_request(request);
    if (rclcpp::spin_until_future_complete(node_, result) != rclcpp::FutureReturnCode::SUCCESS)
    {
      throw std::runtime_error("RosClientCI::getViewSpace failed for unknown reason.");
    }
    
    return ros_conversions::viewSpaceFromMsg(result.get()->viewspace);
  }
  
  RosClientCI::ViewSpaceUpdateResult RosClientCI::addViews(std::vector<View>& new_views)
  {
    auto request = std::make_shared<ig_active_reconstruction_msgs::srv::ViewSpaceUpdate::Request>();
    
    for(const auto& view : new_views)
    {
      request->views.push_back(ros_conversions::viewToMsg(view));
    }
    
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Demanding to add view(s).");
    auto result = views_adder_->async_send_request(request);
    if (rclcpp::spin_until_future_complete(node_, result) != rclcpp::FutureReturnCode::SUCCESS)
    {
      return RosClientCI::ViewSpaceUpdateResult::FAILED;
    }
    
    return ros_conversions::viewSpaceUpdateResultFromMsg(result.get()->update_result);
  }
  
  RosClientCI::ViewSpaceUpdateResult RosClientCI::addView(View new_view)
  {
    auto request = std::make_shared<ig_active_reconstruction_msgs::srv::ViewSpaceUpdate::Request>();
    request->views.push_back(ros_conversions::viewToMsg(new_view));
    
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Demanding to add view.");
    auto result = views_adder_->async_send_request(request);
    if (rclcpp::spin_until_future_complete(node_, result) != rclcpp::FutureReturnCode::SUCCESS)
    {
      return RosClientCI::ViewSpaceUpdateResult::FAILED;
    }
    
    return ros_conversions::viewSpaceUpdateResultFromMsg(result.get()->update_result);
  }
  
  RosClientCI::ViewSpaceUpdateResult RosClientCI::deleteViews(std::vector<View::IdType>& view_ids)
  {
    auto request = std::make_shared<ig_active_reconstruction_msgs::srv::DeleteViews::Request>();
    
    for(const auto& id : view_ids)
    {
      request->ids.push_back(id);
    }
    
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Demanding to delete view(s).");
    auto result = views_deleter_->async_send_request(request);
    if (rclcpp::spin_until_future_complete(node_, result) != rclcpp::FutureReturnCode::SUCCESS)
    {
      return RosClientCI::ViewSpaceUpdateResult::FAILED;
    }
    
    return ros_conversions::viewSpaceUpdateResultFromMsg(result.get()->update_result);
  }
  
  RosClientCI::ViewSpaceUpdateResult RosClientCI::deleteView(View::IdType view_id)
  {
    auto request = std::make_shared<ig_active_reconstruction_msgs::srv::DeleteViews::Request>();
    request->ids.push_back(view_id);
    
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Demanding to delete view.");
    auto result = views_deleter_->async_send_request(request);
    if (rclcpp::spin_until_future_complete(node_, result) != rclcpp::FutureReturnCode::SUCCESS)
    {
      return RosClientCI::ViewSpaceUpdateResult::FAILED;
    }
    
    return ros_conversions::viewSpaceUpdateResultFromMsg(result.get()->update_result);
  }
}
}