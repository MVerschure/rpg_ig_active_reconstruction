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

namespace ros_tools
{
  /*! Template function that throws a ROS fatal error if the parameter was not provided on the server.
   * 
   * @param node Shared pointer to the ROS 2 node
   * @param path Path that will be read from the node
   * @param output (output) Where the parameter will be written
   */
  template<class PARAM_TYPE>
  void getExpParam(const rclcpp::Node::SharedPtr& node, const std::string& path, PARAM_TYPE& output)
  {
    if (!node->get_parameter(path, output))
    {
      RCLCPP_FATAL(rclcpp::get_logger("rclcpp"), "Expected parameter '%s' was not provided on parameter server. Exiting...", path.c_str());
      rclcpp::shutdown();
    }
  }

  /*! Silent template function that loads a parameter or its default value.
   * 
   * @param node Shared pointer to the ROS 2 node
   * @param path Path that will be read from the node
   * @param output (output) Where the parameter will be written
   * @param defaultValue Default value that will be loaded if not provided
   */
  template<class PARAM_TYPE>
  void getParamSilent(const rclcpp::Node::SharedPtr& node, const std::string& path, PARAM_TYPE& output, const PARAM_TYPE& defaultValue)
  {
    rclcpp::Parameter param;
    if (node->get_parameter(path, param))
    {
      output = param.get_value<PARAM_TYPE>();
    }
    else
    {
      output = defaultValue;
    }
  }

  /*! Template function to load a parameter with included default value. If it wasn't found and the default is used, a warning is issued.
   * 
   * @param node Shared pointer to the ROS 2 node
   * @param path Path that will be read from the node
   * @param output (output) Where the parameter will be written
   * @param defaultValue Default value that will be loaded if not provided
   */
  template<class PARAM_TYPE>
  void getParam(const rclcpp::Node::SharedPtr& node, const std::string& path, PARAM_TYPE& output, const PARAM_TYPE& defaultValue)
  {
    rclcpp::Parameter param;
    if (node->get_parameter(path, param))
    {
      output = param.get_value<PARAM_TYPE>();
    }
    else
    {
      output = defaultValue;
      RCLCPP_WARN(rclcpp::get_logger("rclcpp"), "Parameter '%s' not provided, using default: '%s'.", path.c_str(), std::to_string(output).c_str());
    }
  }

  /*! Template function to load a parameter if available. If it wasn't found, the value is not set.
   * 
   * @param node Shared pointer to the ROS 2 node
   * @param path Path that will be read from the node
   * @param output (output) Where the parameter will be written
   */
  template<class PARAM_TYPE>
  void getParamIfAvailable(const rclcpp::Node::SharedPtr& node, const std::string& path, PARAM_TYPE& output)
  {
    rclcpp::Parameter param;
    if (node->get_parameter(path, param))
    {
      output = param.get_value<PARAM_TYPE>();
    }
    else
    {
      RCLCPP_WARN(rclcpp::get_logger("rclcpp"), "Parameter '%s' not provided.", path.c_str());
    }
  }
}