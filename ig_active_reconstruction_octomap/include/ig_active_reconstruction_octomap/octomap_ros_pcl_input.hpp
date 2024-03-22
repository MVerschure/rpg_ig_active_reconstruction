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

#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2/exceptions.h>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include "ig_active_reconstruction_msgs/srv/pcl_input.hpp"

#include "ig_active_reconstruction_octomap/octomap_pcl_input.hpp"

namespace ig_active_reconstruction
{
  
namespace world_representation
{

namespace octomap
{
  
  /*! Class subscribes to ROS pcl topic and feeds it to an octomap::PclInput
   * 
   * Subscribes to "pcl_input" on the passed ros node.
   * Advertices "pcl_input" as service
   */
  template<class TREE_TYPE, class POINTCLOUD_TYPE>
  class RosPclInput
  {
  public:
    /*! Constructor.
     * @param nh ros node handle under which topic and service will be advertised.
     * @param pcl_input PclInput object pointer to which pointclouds are forwarded.
     * @param world_frame Name of the world coordinate frame to which the incoming pointclouds will be transformed.
     */
    RosPclInput( std::shared_ptr<rclcpp::Node> node, std::shared_ptr< PclInput<TREE_TYPE,POINTCLOUD_TYPE> > pcl_input, std::string world_frame );
    
    /*! Add a function that will be called after a new input was processed.
     * @param signal_call The function.
     */
    void addInputDoneSignalCall( boost::function<void()> signal_call );
    
  protected:
    /*! Pcl input topic listener.
     */
    void insertCloudCallback(const std::shared_ptr<sensor_msgs::msg::PointCloud2> cloud);
    
    /*! Pcl input service.S
     */
    bool insertCloudService(
    std::shared_ptr<ig_active_reconstruction_msgs::srv::PclInput::Request> request,
    std::shared_ptr<ig_active_reconstruction_msgs::srv::PclInput::Response> response
    );
    /*! Helper function calling the signal call stack.
     */
    void issueInputDoneSignals();
    
    /*! Inserts a cloud by reference and calls issueInputDoneSignals when done.
     */
    void insertCloud( POINTCLOUD_TYPE& pointcloud );
    
  private:
    std::shared_ptr<rclcpp::Node> node_;
    std::shared_ptr< PclInput<TREE_TYPE,POINTCLOUD_TYPE> > pcl_input_;
    
    std::string world_frame_name_;
    
    std::vector< boost::function<void()> > signal_call_stack_;
    
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pcl_subscriber_;
    rclcpp::Service<ig_active_reconstruction_msgs::srv::PclInput>::SharedPtr pcl_input_service_;
    
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  };
}

}

}

#include "../src/code_base/octomap_ros_pcl_input.inl"