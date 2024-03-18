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

#include <iostream>
#include <string>
#include <thread>
#include <chrono>

#include <boost/thread/thread.hpp>
#include <boost/chrono/include.hpp>

#include <rclcpp/rclcpp.hpp>
#include <ig_active_reconstruction/basic_view_planner.hpp>
#include <ig_active_reconstruction/weighted_linear_utility.hpp>
#include <ig_active_reconstruction/max_calls_termination_criteria.hpp>

#include "ig_active_reconstruction_ros/param_loader.hpp"
#include "ig_active_reconstruction_ros/robot_ros_client_ci.hpp"
#include "ig_active_reconstruction_ros/views_ros_client_ci.hpp"
#include "ig_active_reconstruction_ros/world_representation_ros_client_ci.hpp"

/*! Implements a ROS node holding a BasicViewPlanner, combined with a simple command line user interface.
 */
int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  std::shared_ptr<rclcpp::Node> node = std::make_shared<rclcpp::Node>("basic_view_planner");
  
  namespace iar = ig_active_reconstruction;
  
  // Load parameter configuration
  // ...................................................................................................................
  
  // For the view planner:
  iar::BasicViewPlanner::Config bvp_config;
  bvp_config.discard_visited = false;
  bvp_config.max_visits = -1;
  
  // For the utility calculator
  double cost_weight = 1.0;
  std::vector<std::string> ig_names;
  std::vector<double> ig_weights;
  // Load parameters for cost_weight, ig_names, ig_weights
  
  // For the termination criteria
  unsigned int max_calls = 20;
  // Load parameter for max_calls
  
  // Only the view planner resides here
  // ...................................................................................................................
  iar::BasicViewPlanner view_planner(bvp_config);
  
  // Robot, viewspace module, and world representation are external
  // ...................................................................................................................
  std::shared_ptr<iar::robot::CommunicationInterface> robot_comm = std::make_shared<iar::robot::RosClientCI>(node);
  std::shared_ptr<iar::views::CommunicationInterface> views_comm = std::make_shared<iar::views::RosClientCI>(node);
  std::shared_ptr<iar::world_representation::CommunicationInterface> world_comm = std::make_shared<iar::world_representation::RosClientCI>(node);
  
  view_planner.setRobotCommUnit(robot_comm);
  view_planner.setViewsCommUnit(views_comm);
  view_planner.setWorldCommUnit(world_comm);
  
  // Use the weighted linear utility calculator
  // ...................................................................................................................
  auto utility_calculator = std::make_shared<iar::WeightedLinearUtility>(cost_weight);
  utility_calculator->setRobotCommUnit(robot_comm);
  utility_calculator->setWorldCommUnit(world_comm);
  
  for (unsigned int i = 0; i < ig_names.size() && i < ig_weights.size(); ++i)
  {
    std::cout << "\nUsing information gain '" << ig_names[i] << "' with weight '" << ig_weights[i] << "'.";
    utility_calculator->useInformationGain(ig_names[i], ig_weights[i]);
  }
  
  view_planner.setUtility(utility_calculator);
  
  // Using a simple max. number of calls termination criteria
  // ...................................................................................................................
  auto termination_criteria = std::make_shared<iar::MaxCallsTerminationCriteria>(max_calls);
  
  view_planner.setGoalEvaluationModule(termination_criteria);
  
  // Simple command line user interface
  // ...................................................................................................................
  bool keepReading = true;
  std::function<void()> status_readout = [&view_planner, &keepReading]()
  {    
    while (keepReading)
    {
      iar::BasicViewPlanner::Status status = view_planner.status();
      
      switch (status)
      {
        case iar::BasicViewPlanner::Status::UNINITIALIZED:
          RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "BasicViewPlanner::Status::UNINITIALIZED");
          break;
        case iar::BasicViewPlanner::Status::IDLE:
          RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "BasicViewPlanner::Status::IDLE");
          break;
        case iar::BasicViewPlanner::Status::PAUSED:
          RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "BasicViewPlanner::Status::PAUSED");
          break;
        case iar::BasicViewPlanner::Status::DEMANDING_NEW_DATA:
          RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "BasicViewPlanner::Status::DEMANDING_NEW_DATA");
          break;
        case iar::BasicViewPlanner::Status::DEMANDING_VIEWSPACE:
          RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "BasicViewPlanner::Status::DEMANDING_VIEWSPACE");
          break;
        case iar::BasicViewPlanner::Status::NBV_CALCULATIONS:
          RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "BasicViewPlanner::Status::NBV_CALCULATIONS");
          break;
        case iar::BasicViewPlanner::Status::DEMANDING_MOVE:
          RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "BasicViewPlanner::Status::DEMANDING_MOVE");
          break;
      };
      std::this_thread::sleep_for(std::chrono::seconds(2));
    }
  }; 
  
  std::thread status_reading_thread(status_readout);
  
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Basic View Planner was successfully setup. As soon as other modules are running, we're ready to go.");
  
  std::string gui_info = "\n\n\nBASIC VIEW PLANNER SIMPLE UI\n********************************\nThe following actions are supported ('key toggle'):\n- 'g' (go) Start or unpause view planning.\n- 'p': (pause) Pause procedure.\n- 's' (stop) Stop procedure\n- 'q' (quit) Stop procedure and quit program.\n\n";
  char user_input;
  
  while (true)
  {
    std::cout << gui_info;
    std::cin >> user_input;
    
    switch (user_input)
    {
      case 'g':
        std::cout << "Starting...";
        view_planner.run();
        break;
      case 'p':
        std::cout << "Pausing...";
        view_planner.pause();
        break;
      case 's':
        while (true)
        {
          std::cout << "\n\nAre you sure you want to stop the procedure? (y/n)\n";
          std::cin >> user_input;
          if (user_input == 'y')
          {
            std::cout << "Stopping...";
            view_planner.stop();
            break;
          }
          else if (user_input == 'n')
            break;
        };
        break;
      case 'q':
        while (true)
        {
          std::cout << "\n\nAre you sure you want to quit the program? (y/n)\n";
          std::cin >> user_input;
          if (user_input == 'y')
          {
            std::cout << "Quitting...";
            view_planner.stop();
            return 0;
          }
          else if (user_input == 'n')
            break;
        };
        break;
    };
  }
  
  keepReading = false;
  status_reading_thread.join();
  
  rclcpp::shutdown();
  
  return 0;
}
