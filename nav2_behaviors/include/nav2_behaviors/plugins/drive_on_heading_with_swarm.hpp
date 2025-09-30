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

#ifndef NAV2_BEHAVIORS__PLUGINS__DRIVE_ON_HEADING_WITH_SWARM_HPP_
#define NAV2_BEHAVIORS__PLUGINS__DRIVE_ON_HEADING_WITH_SWARM_HPP_

#include <chrono>
#include <memory>
#include <utility>

#include "nav2_behaviors/plugins/drive_on_heading.hpp"
#include "nav_msgs/msg/path.hpp"
#include "nav2_msgs/action/drive_on_heading_with_swarm.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "amr_interfaces/srv/cut_path_for_vehicles.hpp"

namespace nav2_behaviors
{

/**
 * @class nav2_behaviors::DriveOnHeadingWithSwarm
 * @brief An action server Behavior for driving on heading with swarm path cutting
 */
template<typename ActionT = nav2_msgs::action::DriveOnHeadingWithSwarm>
class DriveOnHeadingWithSwarm : public DriveOnHeading<ActionT>
{
public:
  /**
   * @brief A constructor for nav2_behaviors::DriveOnHeadingWithSwarm
   */
  DriveOnHeadingWithSwarm()
  : DriveOnHeading<ActionT>(),
    waiting_for_path_clear_(false),
    path_check_rate_(2.0)
  {
  }

  ~DriveOnHeadingWithSwarm() = default;

  /**
   * @brief Initialization to run behavior
   * @param command Goal to execute
   * @return Status of behavior
   */
  ResultStatus onRun(const std::shared_ptr<const typename ActionT::Goal> command) override
  {
    ResultStatus status = DriveOnHeading<ActionT>::onRun(command);
    if (status.status != Status::SUCCEEDED) {
      return status;
    }

    waiting_for_path_clear_ = false;
    // has to be old so first check to happens immediately
    last_path_check_time_ = rclcpp::Time(0, 0, this->clock_->get_clock_type());
    current_path_is_cut_ = false;
    path_check_rate_ = command->path_check_rate;

    return status;
  }

  /**
   * @brief Loop function to run behavior
   * @return Status of behavior
   */
  ResultStatus onCycleUpdate() override
  {
    geometry_msgs::msg::PoseStamped current_pose;
    double distance;
    std::unique_ptr<geometry_msgs::msg::TwistStamped> cmd_vel;
    geometry_msgs::msg::Pose2D pose2d;

    ResultStatus status = this->commonCycleUpdateLogic(current_pose, distance, cmd_vel, pose2d);
    if (status.status != Status::RUNNING) {
      return status;
    }

    // Perform collision check before generating path
    if (!this->isCollisionFree(distance, cmd_vel->twist, pose2d)) {
      this->stopRobot();
      RCLCPP_WARN(this->logger_, "Collision Ahead - Exiting DriveOnHeadingWithSwarm");
      return ResultStatus{Status::FAILED, ActionT::Result::COLLISION_AHEAD};
    }

    // Check if it's time to rebuild path and check for cuts
    auto now = this->clock_->now();
    double time_since_last_check = (now - last_path_check_time_).seconds();

    if (time_since_last_check >= (1.0 / path_check_rate_)) {
      last_path_check_time_ = now;

      // Generate path and request cut (blocking)
      nav_msgs::msg::Path path = generateStraightLinePath(current_pose, this->command_x_ - distance, 0.3);
      auto [cut_path, path_cut] = callCutPathForVehiclesService(path);

      if (global_plan_pub_) {
        global_plan_pub_->publish(*cut_path);
      }

      current_path_is_cut_ = path_cut;

      if (path_cut) {
        // First cut
        if (!waiting_for_path_clear_) {
          waiting_for_path_clear_ = true;
          this->stopRobot();
          RCLCPP_WARN(this->logger_, "Path cut detected. Pausing until clear.");
          cut_fail_start_time_ = now;
        }

        // Have been cut for > 30 sec - fail
        if ((now - cut_fail_start_time_).seconds() > 30.0) {
          RCLCPP_ERROR(this->logger_, "Path still cut after 30s. Failing behavior.");
          auto result = std::make_shared<typename ActionT::Result>();
          result->error_code = ActionT::Result::TIMEOUT;
          this->action_server_->terminate_current(result);
          return ResultStatus{Status::FAILED, ActionT::Result::TIMEOUT};
        }
      } else {
        // Path is clear, reset waiting state if needed
        if (waiting_for_path_clear_) {
          waiting_for_path_clear_ = false;
          this->end_time_ = this->clock_->now() + this->command_time_allowance_;
          RCLCPP_INFO(this->logger_, "Path is clear again. Resuming motion.");
        }
      }
    }

    // If path is currently cut, stop and wait
    if (current_path_is_cut_) {
      this->stopRobot();
      return ResultStatus{Status::RUNNING, ActionT::Result::NONE};
    }

    // Publish velocity command
    this->vel_pub_->publish(std::move(cmd_vel));

    return ResultStatus{Status::RUNNING, ActionT::Result::NONE};
  }

protected:
  /**
   * @brief Generate a straight line path for swarm coordination
   * @param current_pose Current robot pose
   * @param dist_to_travel Distance to travel
   * @param poses_distance Distance between poses in the path
   * @return Generated path
   */
  nav_msgs::msg::Path generateStraightLinePath(
    const geometry_msgs::msg::PoseStamped & current_pose,
    const double dist_to_travel,
    const double poses_distance)
  {
    nav_msgs::msg::Path path;
    path.header.frame_id = current_pose.header.frame_id;
    path.header.stamp = this->clock_->now();

    path.poses.push_back(current_pose);
    double yaw = tf2::getYaw(current_pose.pose.orientation);
    double dx = cos(yaw);
    double dy = sin(yaw);
    double direction = (dist_to_travel >= 0.0) ? 1.0 : -1.0;
    int num_poses = static_cast<int>(std::abs(dist_to_travel) / poses_distance);

    for (int i = 1; i <= num_poses; ++i) {
      geometry_msgs::msg::PoseStamped pose = current_pose;
      pose.pose.position.x += dx * poses_distance * i * direction;
      pose.pose.position.y += dy * poses_distance * i * direction;
      path.poses.push_back(pose);
    }

    // Make sure to add the last pose if num poses is decimal
    geometry_msgs::msg::PoseStamped goal_pose = current_pose;
    goal_pose.pose.position.x += dx * dist_to_travel;
    goal_pose.pose.position.y += dy * dist_to_travel;
    path.poses.push_back(goal_pose);

    return path;
  }

  /**
   * @brief Call the CutPathForVehicles service to check for path conflicts
   * @param path Path to check
   * @return Pair of cut path and boolean indicating if path was cut
   */
  std::pair<std::shared_ptr<nav_msgs::msg::Path>, bool> callCutPathForVehiclesService(const nav_msgs::msg::Path & path)
  {
    auto node = this->node_.lock();
    if (!node || !cutpathforvehicles_client_ || !cutpathforvehicles_client_->service_is_ready()) {
      return {std::make_shared<nav_msgs::msg::Path>(path), false};
    }

    auto request = std::make_shared<amr_interfaces::srv::CutPathForVehicles::Request>();
    request->path = path;
    request->fail_if_cut = false;

    auto future = cutpathforvehicles_client_->async_send_request(request);

    // Wait (blocking) for response
    auto start_time = this->clock_->now();
    while (rclcpp::ok() && future.wait_for(std::chrono::milliseconds(10)) != std::future_status::ready) {
      if ((this->clock_->now() - start_time).seconds() > 1.0) {
        RCLCPP_WARN(this->logger_, "CutPathForVehicles service timed out.");
        return {std::make_shared<nav_msgs::msg::Path>(path), false};
      }
    }

    auto response = future.get();
    bool was_cut = response->was_cut;
    auto cut_path_ptr = std::make_shared<nav_msgs::msg::Path>(response->cut_path);
    return {cut_path_ptr, was_cut};
  }

  /**
   * @brief Configuration of behavior action
   */
  void onConfigure() override
  {
    DriveOnHeading<ActionT>::onConfigure();
    auto node = this->node_.lock();
    if (!node) {
      throw std::runtime_error{"Failed to lock node"};
    }

    global_plan_pub_ = node->template create_publisher<nav_msgs::msg::Path>("global_plan", 1);
    cutpathforvehicles_client_ =
      node->template create_client<amr_interfaces::srv::CutPathForVehicles>("cut_path_for_vehicles");
  }

  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr global_plan_pub_;
  rclcpp::Client<amr_interfaces::srv::CutPathForVehicles>::SharedPtr cutpathforvehicles_client_;
  bool waiting_for_path_clear_;
  bool current_path_is_cut_;
  rclcpp::Time cut_fail_start_time_;
  rclcpp::Time last_path_check_time_;
  double path_check_rate_;
};

}  // namespace nav2_behaviors

#endif  // NAV2_BEHAVIORS__PLUGINS__DRIVE_ON_HEADING_WITH_SWARM_HPP_