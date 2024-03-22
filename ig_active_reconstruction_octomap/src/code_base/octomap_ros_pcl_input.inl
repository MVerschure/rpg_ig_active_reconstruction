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

#define TEMPT template<class TREE_TYPE, class POINTCLOUD_TYPE>
#define CSCOPE RosPclInput<TREE_TYPE,POINTCLOUD_TYPE>

#include <pcl/point_types.h>
#include <pcl/conversions.h>
#include <pcl_conversions/pcl_conversions.h>
//#include <pcl_ros/transforms.h>

namespace ig_active_reconstruction
{
  
namespace world_representation
{

namespace octomap
{
  TEMPT
  CSCOPE::RosPclInput(std::shared_ptr<rclcpp::Node> node, std::shared_ptr<PclInput<TREE_TYPE, POINTCLOUD_TYPE>> pcl_input, std::string world_frame)
    : node_(node), pcl_input_(pcl_input), world_frame_name_(world_frame)
    {
        pcl_subscriber_ = node_->create_subscription<sensor_msgs::msg::PointCloud2>(
            "pcl_input", 10, std::bind(&CSCOPE::insertCloudCallback, this, std::placeholders::_1));

        pcl_input_service_ = node_->create_service<ig_active_reconstruction_msgs::srv::PclInput>(
            "pcl_input", std::bind(&CSCOPE::insertCloudService, this, std::placeholders::_1, std::placeholders::_2));

        tf_buffer_ =std::make_unique<tf2_ros::Buffer>(node_->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    }
  
  TEMPT
  void CSCOPE::addInputDoneSignalCall( boost::function<void()> signal_call )
  {
    signal_call_stack_.push_back(signal_call);
  }
  
  TEMPT
  void CSCOPE::insertCloudCallback(const std::shared_ptr<sensor_msgs::msg::PointCloud2> cloud)
  {
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Received new pointcloud. Inserting...");
    POINTCLOUD_TYPE pc;
    pcl::fromROSMsg(*cloud, pc);
    
    insertCloud(pc);
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Inserted new pointcloud");
  }
  
  TEMPT
  bool CSCOPE::insertCloudService(
    std::shared_ptr<ig_active_reconstruction_msgs::srv::PclInput::Request> request,
    std::shared_ptr<ig_active_reconstruction_msgs::srv::PclInput::Response> response
    )
  {
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Received new pointcloud. Inserting...");
    POINTCLOUD_TYPE pc;
    pcl::fromROSMsg(request->pointcloud, pc);
    
    insertCloud(pc);
    
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Inserted new pointcloud");
    response->success = true;
    return true;
  }
  
  TEMPT
  void CSCOPE::issueInputDoneSignals()
  {
    BOOST_FOREACH( boost::function<void()>& call, signal_call_stack_)
    {
      call();
    }
  }
  
  TEMPT
  void CSCOPE::insertCloud( POINTCLOUD_TYPE& pointcloud )
  {
    geometry_msgs::msg::TransformStamped sensor_to_world_tf;
    try
    {
      sensor_to_world_tf = tf_buffer_->lookupTransform(world_frame_name_, pointcloud.header.frame_id, tf2::TimePointZero);
    }
    catch(tf2::TransformException& ex)
    {
      RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "RosPclInput<TREE_TYPE,POINTCLOUD_TYPE>::Transform error of sensor data: ..., quitting callback.");
      return;
    }
    
    Eigen::Matrix4f sensor_to_world;
    //pcl_ros::transformAsMatrix(sensor_to_world_tf, sensor_to_world);
    // Convert the transform to Eigen format
    Eigen::Translation3f t(sensor_to_world_tf.transform.translation.x,
                 sensor_to_world_tf.transform.translation.y,
                 sensor_to_world_tf.transform.translation.z);

    Eigen::Quaternionf q(sensor_to_world_tf.transform.rotation.w,
               sensor_to_world_tf.transform.rotation.x,
               sensor_to_world_tf.transform.rotation.y,
               sensor_to_world_tf.transform.rotation.z);

    sensor_to_world = (t * q).matrix();
    
    Eigen::Transform<double,3,Eigen::Affine> sensor_to_world_transform;
    sensor_to_world_transform = sensor_to_world.cast<double>();
    
    pcl_input_->push(sensor_to_world_transform,pointcloud);
    
    issueInputDoneSignals();
  }
  
}

}

}

#undef CSCOPE
#undef TEMPT

