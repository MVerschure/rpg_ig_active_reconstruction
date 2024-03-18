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

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

namespace ros_tools
{
  /*! Class that listens on a pcl topic and reroutes what it receives to another pcl topic or to a service.
   * Current implementation forwards single packages on demand.
   */
  class PclRerouter : public rclcpp::Node
  {
  public:
    /*! Constructor.
     * @param nh Node handle through which ROS communication is registered
     * @param in_name Input name (Where pcl input is retrieved, relative to node namespace)
     * @param out_name How outputs (srv/topic) are advertised (relative to node namespace)
     */
    PclRerouter(const rclcpp::Node::SharedPtr node, const std::string& in_name = "in", const std::string& out_name = "out" );
    
    /*! Reroutes the next incoming pointcloud to the output.
     * @param max_wait_time Max wait time before rerouting is considered to have failed.
     * @return True if a data packet was rerouted.
     */
    bool rerouteOneToTopic(std::chrono::seconds max_wait_time = std::chrono::seconds(1));
    
    /*! Reroutes the next incoming pointcloud to the service.
     * @param max_wait_time Max wait time before rerouting is considered to have failed.
     * @return True if a data packet was rerouted.
     */
    bool rerouteOneToSrv();
    
  protected:
    /*! Called for incoming pointclouds.
     */
    void pclCallback( const sensor_msgs::msg::PointCloud2::SharedPtr msg );
    
  protected:
    rclcpp::Node::SharedPtr node_;

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pcl_subscriber_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pcl_publisher_;
    rclcpp::Client<sensor_msgs::srv::PointCloud2>::SharedPtr pcl_service_caller_;
    
    bool forward_one_;
    bool has_published_one_;
    
    bool one_to_srv_;
    bool service_response_;
  };
  
}