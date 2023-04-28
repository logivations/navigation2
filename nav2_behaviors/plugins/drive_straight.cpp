// Copyright (c) 2022 Joshua Wallace
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

#include "drive_straight.hpp"

namespace nav2_behaviors
{

Status DriveStraight::onRun(const std::shared_ptr<const DriveStraightAction::Goal> command)
{
  if (command->target.y != 0.0 || command->target.z != 0.0) {
    RCLCPP_INFO(
      logger_,
      "Backing up in Y and Z not supported, will only move in X.");
    return Status::FAILED;
  }

  command_x_ = command->target.x;
  command_speed_ = std::fabs(command->speed);
  free_goal_vel = command->free_goal_vel;
  check_local_costmap = command->check_local_costmap;
  command_time_allowance_ = command->time_allowance;

  end_time_ = steady_clock_.now() + command_time_allowance_;

  if (!nav2_util::getCurrentPose(
      initial_pose_, *tf_, global_frame_, robot_base_frame_,
      transform_tolerance_))
  {
    RCLCPP_ERROR(logger_, "Initial robot pose is not available.");
    return Status::FAILED;
  }

  return Status::SUCCEEDED;
}

}  // namespace nav2_behaviors

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(nav2_behaviors::DriveStraight, nav2_core::Behavior)
