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

#include <string>
#include <memory>

#include "nav2_behavior_tree/plugins/action/drive_on_heading_with_swarm_action.hpp"

namespace nav2_behavior_tree
{

DriveOnHeadingWithSwarmAction::DriveOnHeadingWithSwarmAction(
  const std::string & xml_tag_name,
  const std::string & action_name,
  const BT::NodeConfiguration & conf)
: BtActionNode<nav2_msgs::action::DriveOnHeadingWithSwarm>(xml_tag_name, action_name, conf)
{
}

void DriveOnHeadingWithSwarmAction::initialize()
{
  double dist;
  getInput("dist_to_travel", dist);
  double speed;
  getInput("speed", speed);
  double time_allowance;
  getInput("time_allowance", time_allowance);
  bool free_goal_vel;
  getInput("free_goal_vel", free_goal_vel);
  float path_check_rate;
  getInput("path_check_rate", path_check_rate);

  // Populate the input message
  goal_.target.x = dist;
  goal_.target.y = 0.0;
  goal_.target.z = 0.0;
  goal_.speed = speed;
  goal_.time_allowance = rclcpp::Duration::from_seconds(time_allowance);
  goal_.free_goal_vel = free_goal_vel;
  goal_.path_check_rate = path_check_rate;
}

void DriveOnHeadingWithSwarmAction::on_tick()
{
  if (!BT::isStatusActive(status())) {
    initialize();
  }
}

BT::NodeStatus DriveOnHeadingWithSwarmAction::on_success()
{
  setOutput("error_code_id", ActionResult::NONE);
  return BT::NodeStatus::SUCCESS;
}

BT::NodeStatus DriveOnHeadingWithSwarmAction::on_aborted()
{
  setOutput("error_code_id", result_.result->error_code);
  return BT::NodeStatus::FAILURE;
}

BT::NodeStatus DriveOnHeadingWithSwarmAction::on_cancelled()
{
  setOutput("error_code_id", ActionResult::NONE);
  return BT::NodeStatus::SUCCESS;
}

}  // namespace nav2_behavior_tree

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  BT::NodeBuilder builder =
    [](const std::string & name, const BT::NodeConfiguration & config)
    {
      return std::make_unique<nav2_behavior_tree::DriveOnHeadingWithSwarmAction>(
        name, "drive_on_heading_with_swarm", config);
    };

  factory.registerBuilder<nav2_behavior_tree::DriveOnHeadingWithSwarmAction>(
    "DriveOnHeadingWithSwarm", builder);
}