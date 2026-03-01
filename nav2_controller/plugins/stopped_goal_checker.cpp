/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2017, Locus Robotics
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the copyright holder nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 */

#include <cmath>
#include <string>
#include <memory>
#include <limits>
#include <vector>
#include "nav2_controller/plugins/stopped_goal_checker.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "nav2_util/node_utils.hpp"

using std::hypot;
using std::fabs;

using rcl_interfaces::msg::ParameterType;
using std::placeholders::_1;

namespace nav2_controller
{

StoppedGoalChecker::StoppedGoalChecker()
: SimpleGoalChecker(), rot_stopped_velocity_(0.25), trans_stopped_velocity_(0.25)
{
}

void StoppedGoalChecker::initialize(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
  const std::string & plugin_name,
  const std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
{
  plugin_name_ = plugin_name;
  SimpleGoalChecker::initialize(parent, plugin_name, costmap_ros);

  auto node = parent.lock();

  nav2_util::declare_parameter_if_not_declared(
    node,
    plugin_name + ".rot_stopped_velocity", rclcpp::ParameterValue(0.25));
  nav2_util::declare_parameter_if_not_declared(
    node,
    plugin_name + ".trans_stopped_velocity", rclcpp::ParameterValue(0.25));

  node->get_parameter(plugin_name + ".rot_stopped_velocity", rot_stopped_velocity_);
  node->get_parameter(plugin_name + ".trans_stopped_velocity", trans_stopped_velocity_);

  // Add callback for dynamic parameters
  dyn_params_handler_ = node->add_on_set_parameters_callback(
    std::bind(&StoppedGoalChecker::dynamicParametersCallback, this, _1));
}

bool StoppedGoalChecker::isGoalReached(
  const geometry_msgs::msg::Pose & query_pose, const geometry_msgs::msg::Pose & goal_pose,
  const geometry_msgs::msg::Twist & velocity)
{
  double x_in_goal_frame = 0.0;
  double y_in_goal_frame = 0.0;

  if(std::isfinite(x_goal_tolerance_) || std::isfinite(y_goal_tolerance_)){
    // Transform to goal frame
    tf2::Transform goal_frame_transform;
    goal_frame_transform.setOrigin(tf2::Vector3(goal_pose.position.x, goal_pose.position.y, 0.0));
    goal_frame_transform.setRotation(tf2::Quaternion(goal_pose.orientation.x, goal_pose.orientation.y, goal_pose.orientation.z, goal_pose.orientation.w));

    tf2::Transform query_pose_in_goal_frame = goal_frame_transform.inverse() * 
      tf2::Transform(tf2::Quaternion(query_pose.orientation.x, query_pose.orientation.y, query_pose.orientation.z, query_pose.orientation.w),
                    tf2::Vector3(query_pose.position.x, query_pose.position.y, query_pose.position.z));

    x_in_goal_frame = fabs(query_pose_in_goal_frame.getOrigin().x());
    y_in_goal_frame = fabs(query_pose_in_goal_frame.getOrigin().y());
  }

  bool ret = SimpleGoalChecker::isGoalReached(query_pose, goal_pose, velocity);
  if (!ret) {
    return ret;
  }

  bool within_x_tolerance = x_in_goal_frame <= x_goal_tolerance_;
  bool within_y_tolerance = y_in_goal_frame <= y_goal_tolerance_;

  return fabs(velocity.angular.z) <= rot_stopped_velocity_ &&
         hypot(velocity.linear.x, velocity.linear.y) <= trans_stopped_velocity_ && within_x_tolerance && within_y_tolerance;
}

bool StoppedGoalChecker::getTolerances(
  geometry_msgs::msg::Pose & pose_tolerance,
  geometry_msgs::msg::Twist & vel_tolerance)
{
  double invalid_field = std::numeric_limits<double>::lowest();

  // populate the poses
  bool rtn = SimpleGoalChecker::getTolerances(pose_tolerance, vel_tolerance);

  // override the velocities
  vel_tolerance.linear.x = trans_stopped_velocity_;
  vel_tolerance.linear.y = trans_stopped_velocity_;
  vel_tolerance.linear.z = invalid_field;

  vel_tolerance.angular.x = invalid_field;
  vel_tolerance.angular.y = invalid_field;
  vel_tolerance.angular.z = rot_stopped_velocity_;

  return true && rtn;
}

rcl_interfaces::msg::SetParametersResult
StoppedGoalChecker::dynamicParametersCallback(std::vector<rclcpp::Parameter> parameters)
{
  rcl_interfaces::msg::SetParametersResult result;
  for (auto parameter : parameters) {
    const auto & param_type = parameter.get_type();
    const auto & param_name = parameter.get_name();
    if (param_name.find(plugin_name_ + ".") != 0) {
      continue;
    }

    if (param_type == ParameterType::PARAMETER_DOUBLE) {
      if (param_name == plugin_name_ + ".rot_stopped_velocity") {
        rot_stopped_velocity_ = parameter.as_double();
      } else if (param_name == plugin_name_ + ".trans_stopped_velocity") {
        trans_stopped_velocity_ = parameter.as_double();
      }
    }
  }
  result.successful = true;
  return result;
}

}  // namespace nav2_controller

PLUGINLIB_EXPORT_CLASS(nav2_controller::StoppedGoalChecker, nav2_core::GoalChecker)
