// Copyright (c) 2020 Pablo Inigo Blasco
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

#include <string>
#include <memory>
#include <functional>

#include "behaviortree_cpp_v3/decorator_node.h"
#include "nav2_util/lifecycle_node.hpp"

#include "nav2_behavior_tree/plugins/decorator/planner_selector.hpp"

#include "rclcpp/rclcpp.hpp"

#define DEFAULT_PLANNER_ID "GridBased"
#define DEFAULT_CONTROLLER_ID "FollowPath"

#define PLANNER_PARAM_NAME "PlannerSelector.planner_id"
#define CONTROLLER_PARAM_NAME "PlannerSelector.controller_id"

#define DEFAULT_PLANNER_BT_INPUT "default_planner_id"
#define DEFAULT_CONTROLLER_BT_INPUT "default_controller_id"

#define PLANNER_ID_BLACKBOARD  "desired_planner_id"
#define CONTROLLER_ID_BLACKBOARD  "desired_controller_id"

namespace nav2_behavior_tree
{

using std::placeholders::_1;

PlannerSelector::PlannerSelector(
  const std::string & name,
  const BT::NodeConfiguration & conf)
: BT::DecoratorNode(name, conf)
{
  auto logger = rclcpp::get_logger("planner_selector");
  RCLCPP_WARN_STREAM(rclcpp::get_logger("planner_selector"), "Planner selector constructor ");

  auto node = config().blackboard->get<nav2_util::LifecycleNode::SharedPtr>("bt_navigator_node");
  RCLCPP_WARN_STREAM(rclcpp::get_logger("planner_selector"), "Planner selector Start, Node: " << node->get_name());

  planner_id_ = DEFAULT_PLANNER_ID;
  controller_id_ = DEFAULT_CONTROLLER_ID;

  // get parameter values from ros2 node parameters
  node->declare_parameter(PLANNER_PARAM_NAME, planner_id_);
  node->declare_parameter(CONTROLLER_PARAM_NAME, controller_id_);

  // priority for default planners parameters: 
  // first: initial ros2 parameter on staartup
  // second: navigation bt xml

  //------- Default planner configuration -------------------  
  bool udpate_ros2_parameter = false;
  if(getInput(DEFAULT_PLANNER_BT_INPUT, planner_id_))
  {
    RCLCPP_INFO_STREAM(logger, "PlannerSelector, default planner_id was specified in the bt node input in bt xml '"<< DEFAULT_PLANNER_BT_INPUT << "': " << planner_id_);
    udpate_ros2_parameter  = true;
  }
  else if(node->get_parameter(PLANNER_PARAM_NAME, planner_id_))
  {
    RCLCPP_WARN_STREAM(logger, "PlannerSelector, default planner_id was specified as ros2 parameter '" << PLANNER_PARAM_NAME << "': " << planner_id_);  
  }
  else
  {
    RCLCPP_WARN_STREAM(logger, "PlannerSelector, default planner_id was not specified neither as ros2 parameter '" << PLANNER_PARAM_NAME << "' nor bt node input in bt xml '" << DEFAULT_PLANNER_BT_INPUT << "'. Using by default '" << planner_id_ << "'");
    udpate_ros2_parameter  = true;
  }

  if(udpate_ros2_parameter)
  {
    RCLCPP_WARN_STREAM(logger, "updating planner id to:" <<planner_id_ );
    node->set_parameter(rclcpp::Parameter(PLANNER_PARAM_NAME, planner_id_));
  }

  //------- Default controller configuration -------------------  
  udpate_ros2_parameter = false;
  if(getInput(DEFAULT_CONTROLLER_BT_INPUT, controller_id_))
  {
    RCLCPP_INFO_STREAM(logger, "PlannerSelector, default controller_id was specified in the bt node input in bt xml '" << DEFAULT_CONTROLLER_BT_INPUT << "': " << controller_id_);
    udpate_ros2_parameter  = true;
  }
  else if(node->get_parameter(CONTROLLER_PARAM_NAME, controller_id_))
  {
    RCLCPP_WARN_STREAM(logger, "PlannerSelector, default controller_id was specified as ros2 parameter '" << CONTROLLER_PARAM_NAME << "': " << controller_id_);  
  }
  else
  {
    RCLCPP_WARN_STREAM(logger, "PlannerSelector, default controller_id was not specified neither as ros2 parameter '" << CONTROLLER_PARAM_NAME << "' nor bt node input in bt xml '" << DEFAULT_CONTROLLER_BT_INPUT"'. Using by default '" << controller_id_ << "'");
    udpate_ros2_parameter  = true;
  }

  if(udpate_ros2_parameter)
  {
    RCLCPP_WARN_STREAM(logger, "updating controller id id to:" <<planner_id_ );
    node->set_parameter(rclcpp::Parameter(CONTROLLER_PARAM_NAME, controller_id_));
  }
  //-----------------------------

  // blackboard communication with planner_server and controller_server
  config().blackboard->set(PLANNER_ID_BLACKBOARD, planner_id_);
  config().blackboard->set(CONTROLLER_ID_BLACKBOARD, controller_id_);

  // serve dynamic updates via ros2 parameters
  param_update_callback_handle = node->add_on_set_parameters_callback(std::bind(&PlannerSelector::on_parameters_set, this,_1));
}

rcl_interfaces::msg::SetParametersResult PlannerSelector::on_parameters_set(const std::vector<rclcpp::Parameter> & updated_parameters)
{
  auto logger = rclcpp::get_logger("planner_selector");
  RCLCPP_INFO_STREAM(logger, "PlannerSelector::on_parameters_set");
  rcl_interfaces::msg::SetParametersResult result;

  result.successful = false;
  for(auto& p: updated_parameters)
  {
    auto pname = p.get_name();
    if(pname == PLANNER_PARAM_NAME)
    {
      config().blackboard->set(PLANNER_ID_BLACKBOARD, p.as_string());
      RCLCPP_INFO_STREAM(logger, "Planner id updated to:" << p.as_string() );
      result.successful = true;
    }
    else if(pname == CONTROLLER_PARAM_NAME)
    {
      config().blackboard->set(CONTROLLER_ID_BLACKBOARD, p.as_string());
      RCLCPP_INFO_STREAM(logger, "Controller id updated to:" << p.as_string() );
      result.successful = true;
    }
  }

  if(result.successful)
  {
    result.reason = "success";
  }
  else
  {
    
    result.reason = "no planner updated";
  }

  return result;
}

inline BT::NodeStatus PlannerSelector::tick()
{
  return child_node_->executeTick();
}

}

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<nav2_behavior_tree::PlannerSelector>("PlannerSelector");
}
