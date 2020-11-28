// Copyright (c) 2018 Intel Corporation
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <memory>
#include <string>

#include "nav2_behavior_tree/plugins/action/follow_path_action.hpp"

namespace nav2_behavior_tree
{

FollowPathAction::FollowPathAction(
  const std::string & xml_tag_name,
  const std::string & action_name,
  const BT::NodeConfiguration & conf)
: BtActionNode<nav2_msgs::action::FollowPath>(xml_tag_name, action_name, conf)
{
}

void FollowPathAction::on_tick()
{
  goal_.controller_id = "";
  getInput("path", goal_.path);
  getInput("controller_id", goal_.controller_id);
  

  RCLCPP_INFO_STREAM(rclcpp::get_logger("FollowPathAction"), "controller from input: " << goal_.controller_id);

  // controller selector 
  if(goal_.controller_id == "")
  {
    
    auto controller_selector_desired_controller = config().blackboard->get<std::string>("desired_controller_id");
    RCLCPP_INFO_STREAM(rclcpp::get_logger("FollowPathAction"), "controller from blackboard 'desired_controller_id': " << controller_selector_desired_controller);
    if (controller_selector_desired_controller!= "")
    {
      goal_.controller_id = controller_selector_desired_controller;
      RCLCPP_DEBUG_STREAM(rclcpp::get_logger("FollowPathAction"), "controller successfuly selected: " << controller_selector_desired_controller);   
    }
    else
    {
      RCLCPP_DEBUG_STREAM(rclcpp::get_logger("FollowPathAction"), "controller selection failure. controller_id was not provided neither: from btNode input 'controller_id' nor controller selector ros2 param 'PlannerSelector.controller_id'");
    }
  }
}

void FollowPathAction::on_wait_for_result()
{
  // Grab the new path
  nav_msgs::msg::Path new_path;
  getInput("path", new_path);

  // Check if it is not same with the current one
  if (goal_.path != new_path) {
    // the action server on the next loop iteration
    goal_.path = new_path;
    goal_updated_ = true;
  }
}

}  // namespace nav2_behavior_tree

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  BT::NodeBuilder builder =
    [](const std::string & name, const BT::NodeConfiguration & config)
    {
      return std::make_unique<nav2_behavior_tree::FollowPathAction>(
        name, "follow_path", config);
    };

  factory.registerBuilder<nav2_behavior_tree::FollowPathAction>(
    "FollowPath", builder);
}
