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

#pragma once

#include "rclcpp/rclcpp.hpp"
#include "ig_active_reconstruction/robot_communication_interface.hpp"

#include "ig_active_reconstruction_msgs/srv/view_request.hpp"
#include "ig_active_reconstruction_msgs/srv/retrieve_data.hpp"
#include "ig_active_reconstruction_msgs/srv/movement_cost_calculation.hpp"
#include "ig_active_reconstruction_msgs/srv/move_to_order.hpp"

namespace ig_active_reconstruction
{
namespace robot
{
  
  /*! Generic remote ROS client robot communication interface implementation.
   * 
   * Uses the ROS communication interface (topics, services etc.) to forward requests.
   */
  class RosClientCI: public CommunicationInterface
  {
  public:
    /*! Constructor
     * @param nh_sub ROS node handle defines the namespace in which ROS communication will be carried out for any topic or service subscribers.
     */
    RosClientCI( rclcpp::Node::SharedPtr node);
  
    /*! returns the current view */
    virtual views::View getCurrentView();
    
    /*! Commands robot to retrieve new data.
     * 
     * @throws std::runtime_error If receiving data failed.
     * @return information about what happened (data received, receival failed )
     */
    virtual ReceptionInfo retrieveData();
    
    /*! Returns the cost to move from the current view to the indicated view
     * 
     * @throws std::runtime_error If receiving data failed.
     * @param target_view the next view
     * @return cost to move to that view
     */
    virtual MovementCost movementCost(views::View& target_view);
    
    /*! returns the cost to move from start view to target view
     * @param start_view the start view
     * @param target_view the target view
     * @param fill_additional_information if true then the different parts of the cost will be included in the additional fields as well
     * @return cost for the movement
     */
    virtual MovementCost movementCost(views::View& start_view, views::View& target_view, bool fill_additional_information);
    
    /*! Tells the robot to get the camera to a new view
     * @param _target_view where to move to
     * @return false if the operation failed
     */
    virtual bool moveTo(views::View& target_view);
    
  protected:
    rclcpp::Node::SharedPtr node_;
    
    rclcpp::Client<ig_active_reconstruction_msgs::srv::ViewRequest>::SharedPtr current_view_retriever_;
    rclcpp::Client<ig_active_reconstruction_msgs::srv::RetrieveData>::SharedPtr data_retriever_;
    rclcpp::Client<ig_active_reconstruction_msgs::srv::MovementCostCalculation>::SharedPtr cost_retriever_;
    rclcpp::Client<ig_active_reconstruction_msgs::srv::MoveToOrder>::SharedPtr robot_mover_;
  };
  
}
}