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

#include "rclcpp/rclcpp.hpp"
#include "ig_active_reconstruction_ros/param_loader.hpp"
#include "ig_active_reconstruction/views_simple_view_space_module.hpp"
#include "ig_active_reconstruction_ros/views_ros_server_ci.hpp"

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::Node::SharedPtr node = std::make_shared<rclcpp::Node>("simple_viewspace_module");
  
  namespace iar = ig_active_reconstruction;
  
  // Load configuration
  //-----------------------------------------------------------------------------------------
  // Hardcoding it because parameters feel like a hassle.
  std::string viewspace_file_path = "/home/mink/Desktop/SAMXL/active_reconstruction_ws/src/rpg_ig_active_reconstruction/example/flying_gazebo_stereo_cam/config/dome_48_views.txt";
  
  //std::string viewspace_file_path;
  //rclcpp::Parameter::Parameter("viewspace_file_path", viewspace_file_path);
  
  // Instantiate viewspace module
  //-----------------------------------------------------------------------------------------
  auto viewspace_module = std::make_shared<iar::views::SimpleViewSpaceModule>(viewspace_file_path);
  
  // Expose the viewspace module to ROS
  //-----------------------------------------------------------------------------------------
  iar::views::RosServerCI comm_unit(node, viewspace_module);
  
  // Spin
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "simple_viewspace_module is ready");
  rclcpp::spin(node);
  
  rclcpp::shutdown();
  
  return 0;
}