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

#include "nav2_behavior_tree/plugins/action/compute_path_to_pose_action.hpp"

namespace nav2_behavior_tree
{

ComputePathToPoseAction::ComputePathToPoseAction(
  const std::string & xml_tag_name,
  const std::string & action_name,
  const BT::NodeConfiguration & conf)
: BtActionNode<nav2_msgs::action::ComputePathToPose>(xml_tag_name, action_name, conf)
{
}

void ComputePathToPoseAction::on_tick()
{
  goal_.planner_id = "";
  getInput("goal", goal_.pose);
  getInput("planner_id", goal_.planner_id);

  RCLCPP_DEBUG_STREAM(rclcpp::get_logger("ComputePathToPoseAction"), "received planner id: " << goal_.planner_id );

  // controller selector 
  if(goal_.planner_id == "")
  {
    auto planner_selector_desired_controller = config().blackboard->get<std::string>("desired_planner_id");
    RCLCPP_DEBUG_STREAM(rclcpp::get_logger("ComputePathToPoseAction"), "planner from ros2 parameter: " << planner_selector_desired_controller);
    if (planner_selector_desired_controller!= "")
    {
      RCLCPP_DEBUG_STREAM(rclcpp::get_logger("ComputePathToPoseAction"), "planner successfuly selected: " << planner_selector_desired_controller);
      goal_.planner_id = planner_selector_desired_controller;
    }
    else
    {
      RCLCPP_DEBUG_STREAM(rclcpp::get_logger("ComputePathToPoseAction"), "planner selection failure. planner_id was not provided neither: from btNode input 'planner_id' nor planner selector ros2 param 'PlannerSelector.planner_id'");
    }
  }
}

BT::NodeStatus ComputePathToPoseAction::on_success()
{
  setOutput("path", result_.result->path);
  return BT::NodeStatus::SUCCESS;
}

}  // namespace nav2_behavior_tree

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  BT::NodeBuilder builder =
    [](const std::string & name, const BT::NodeConfiguration & config)
    {
      return std::make_unique<nav2_behavior_tree::ComputePathToPoseAction>(
        name, "compute_path_to_pose", config);
    };

  factory.registerBuilder<nav2_behavior_tree::ComputePathToPoseAction>(
    "ComputePathToPose", builder);
}
