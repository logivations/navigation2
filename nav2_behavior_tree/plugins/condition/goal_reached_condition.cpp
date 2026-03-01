// Copyright (c) 2019 Intel Corporation
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

#include "nav2_util/robot_utils.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_ros_common/node_utils.hpp"
#include "tf2/utils.h"
#include "angles/angles.h"

#include "nav2_behavior_tree/plugins/condition/goal_reached_condition.hpp"

namespace nav2_behavior_tree
{

GoalReachedCondition::GoalReachedCondition(
  const std::string & condition_name,
  const BT::NodeConfiguration & conf)
: BT::ConditionNode(condition_name, conf)
{
  auto node = config().blackboard->get<nav2::LifecycleNode::SharedPtr>("node");

  robot_base_frame_ = BT::deconflictPortAndParamFrame<std::string>(
    node, "robot_base_frame", this);
}

GoalReachedCondition::~GoalReachedCondition()
{
  cleanup();
}

void GoalReachedCondition::initialize()
{
  node_ = config().blackboard->get<nav2::LifecycleNode::SharedPtr>("node");

  goal_reached_tol_ = node_->declare_or_get_parameter("goal_reached_tol", 0.25);

  getInput("xy_goal_tolerance", goal_reached_tol_);
  getInput("x_goal_tolerance", goal_reached_tol_x_);
  getInput("y_goal_tolerance", goal_reached_tol_y_);
  getInput("yaw_goal_tolerance", goal_reached_tol_yaw_);
  tf_ = config().blackboard->get<std::shared_ptr<tf2_ros::Buffer>>("tf_buffer");

  node_->get_parameter("transform_tolerance", transform_tolerance_);
}

BT::NodeStatus GoalReachedCondition::tick()
{
  if (!BT::isStatusActive(status())) {
    initialize();
  }

  if (isGoalReached()) {
    return BT::NodeStatus::SUCCESS;
  }
  return BT::NodeStatus::FAILURE;
}

bool GoalReachedCondition::isGoalReached()
{
  geometry_msgs::msg::PoseStamped goal;
  getInput("goal", goal);

  geometry_msgs::msg::PoseStamped current_pose;
  if (!nav2_util::getCurrentPose(
      current_pose, *tf_, goal.header.frame_id, robot_base_frame_, transform_tolerance_))
  {
    RCLCPP_DEBUG(node_->get_logger(), "Current robot pose is not available.");
    return false;
  }

  double x_in_goal_frame = 0.0;
  double y_in_goal_frame = 0.0;

  if(std::isfinite(goal_reached_tol_x_) || std::isfinite(goal_reached_tol_y_)){
    // Transform to goal frame
    tf2::Transform goal_frame_transform;
    goal_frame_transform.setOrigin(tf2::Vector3(goal.pose.position.x, goal.pose.position.y, 0.0));
    goal_frame_transform.setRotation(tf2::Quaternion(goal.pose.orientation.x, goal.pose.orientation.y, goal.pose.orientation.z, goal.pose.orientation.w));

    tf2::Transform current_pose_in_goal_frame = goal_frame_transform.inverse() * tf2::Transform(tf2::Quaternion(current_pose.pose.orientation.x, current_pose.pose.orientation.y, current_pose.pose.orientation.z, current_pose.pose.orientation.w),
    tf2::Vector3(current_pose.pose.position.x, current_pose.pose.position.y, current_pose.pose.position.z));

    x_in_goal_frame = fabs(current_pose_in_goal_frame.getOrigin().x());
    y_in_goal_frame = fabs(current_pose_in_goal_frame.getOrigin().y());
  }

  double dx = goal.pose.position.x - current_pose.pose.position.x;
  double dy = goal.pose.position.y - current_pose.pose.position.y;
  double current_yaw = tf2::getYaw(current_pose.pose.orientation);
  double goal_yaw = tf2::getYaw(goal.pose.orientation);
  double dangle = fabs(angles::shortest_angular_distance(goal_yaw, current_yaw));

  // Check conditions for x, y, and xy tolerances
  bool within_xy_tolerance = (dx * dx + dy * dy) <= (goal_reached_tol_ * goal_reached_tol_);
  bool within_x_tolerance = x_in_goal_frame <= goal_reached_tol_x_;
  bool within_y_tolerance = y_in_goal_frame <= goal_reached_tol_y_;
  bool within_yaw_tolerance = dangle <= goal_reached_tol_yaw_;

  return within_xy_tolerance && within_x_tolerance && within_y_tolerance && within_yaw_tolerance;
}

}  // namespace nav2_behavior_tree

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<nav2_behavior_tree::GoalReachedCondition>("GoalReached");
}
