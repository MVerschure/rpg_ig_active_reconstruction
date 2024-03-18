#pragma once

#include "rclcpp/rclcpp.hpp"
#include "ig_active_reconstruction/views_communication_interface.hpp"

#include "ig_active_reconstruction_msgs/srv/delete_views.hpp"
#include "ig_active_reconstruction_msgs/srv/view_space_request.hpp"
#include "ig_active_reconstruction_msgs/srv/view_space_update.hpp"

namespace ig_active_reconstruction
{
  
namespace views
{
  
  class RosServerCI : public CommunicationInterface
  {
  public:
    RosServerCI(rclcpp::Node::SharedPtr node, std::shared_ptr<CommunicationInterface> linked_interface);
    
    virtual const ViewSpace& getViewSpace();
    
    virtual ViewSpaceUpdateResult addViews(std::vector<View>& new_views);
    
    virtual ViewSpaceUpdateResult addView(View new_view);
    
    virtual ViewSpaceUpdateResult deleteViews(std::vector<View::IdType>& view_ids);
    
    virtual ViewSpaceUpdateResult deleteView(View::IdType view_id);
    
  protected:
    bool viewspaceService(const std::shared_ptr<ig_active_reconstruction_msgs::srv::ViewSpaceRequest::Request> request, 
                          std::shared_ptr<ig_active_reconstruction_msgs::srv::ViewSpaceRequest::Response> response);
    
    bool viewsAdderService(const std::shared_ptr<ig_active_reconstruction_msgs::srv::ViewSpaceUpdate::Request> request,
                          std::shared_ptr<ig_active_reconstruction_msgs::srv::ViewSpaceUpdate::Response> response);
    
    bool viewsDeleterService(const std::shared_ptr<ig_active_reconstruction_msgs::srv::DeleteViews::Request> request,
                          std::shared_ptr<ig_active_reconstruction_msgs::srv::DeleteViews::Response> response);
    
  protected:
    rclcpp::Node::SharedPtr node_;
    std::shared_ptr<CommunicationInterface> linked_interface_;
    
    rclcpp::Service<ig_active_reconstruction_msgs::srv::ViewSpaceRequest>::SharedPtr viewspace_service_;
    rclcpp::Service<ig_active_reconstruction_msgs::srv::ViewSpaceUpdate>::SharedPtr views_adder_service_;
    rclcpp::Service<ig_active_reconstruction_msgs::srv::DeleteViews>::SharedPtr views_deleter_service_;
  };
  
}

}