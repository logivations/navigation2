// Copyright (c) 2026 Logivations GmbH
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

// A FollowPath goal that is reached already when it arrives must not get a control step.
// miele-amr2 2026-11-05 08:48-08:53 UTC: the BT re-sent a path that was only the robot's own
// pose every tick; each goal got one TEB step (v +-0.01 m/s, w 0.075 rad/s) before "Reached the
// goal!", a 20 Hz stream that swung the tricycle's wheel between +-80 deg for 4.5 min.

#include <atomic>
#include <chrono>
#include <cstdio>
#include <fstream>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include <unistd.h>

#include "ament_index_cpp/get_package_prefix.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "gtest/gtest.h"
#include "nav2_controller/controller_server.hpp"
#include "nav2_core/controller.hpp"
#include "nav2_msgs/action/follow_path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "tf2_ros/static_transform_broadcaster.hpp"

using namespace std::chrono_literals;
using FollowPath = nav2_msgs::action::FollowPath;

namespace
{

// Stands in for TEB: always asks for the tiny turning command TEB gave at the robot's own pose
class RecordingController : public nav2_core::Controller
{
public:
  void configure(
    const nav2::LifecycleNode::WeakPtr &, std::string, std::shared_ptr<tf2_ros::Buffer>,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS>) override {}
  void cleanup() override {}
  void activate() override {}
  void deactivate() override {}
  void newPathReceived(const nav_msgs::msg::Path &) override {}
  void setSpeedLimit(const double &, const bool &) override {}

  geometry_msgs::msg::TwistStamped computeVelocityCommands(
    const geometry_msgs::msg::PoseStamped &, const geometry_msgs::msg::Twist &,
    nav2_core::GoalChecker *, const nav_msgs::msg::Path &,
    const geometry_msgs::msg::PoseStamped &) override
  {
    calls++;
    geometry_msgs::msg::TwistStamped cmd;
    cmd.twist.linear.x = 0.01;
    cmd.twist.angular.z = 0.075;
    return cmd;
  }

  std::atomic<int> calls{0};
};

class TestControllerServer : public nav2_controller::ControllerServer
{
public:
  void replaceController(const std::string & id, nav2_core::Controller::Ptr controller)
  {
    controllers_[id] = controller;
  }
};

geometry_msgs::msg::PoseStamped odomPose(double x)
{
  geometry_msgs::msg::PoseStamped pose;
  pose.header.frame_id = "odom";
  pose.pose.position.x = x;
  pose.pose.orientation.w = 1.0;
  return pose;
}

class GoalReachedBeforeControlTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    // Only a placeholder that lets the server configure before the test swaps in its own
    // controller; not a package dependency (it test-depends on nav2_controller itself)
    try {
      ament_index_cpp::get_package_prefix("nav2_regulated_pure_pursuit_controller");
    } catch (const ament_index_cpp::PackageNotFoundError &) {
      GTEST_SKIP() << "nav2_regulated_pure_pursuit_controller is not installed";
    }

    client_node_ = std::make_shared<rclcpp::Node>("follow_path_test_client");
    tf_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(client_node_);
    geometry_msgs::msg::TransformStamped odom_to_base;
    odom_to_base.header.stamp = client_node_->now();
    odom_to_base.header.frame_id = "odom";
    odom_to_base.child_frame_id = "base_link";
    odom_to_base.transform.rotation.w = 1.0;
    tf_broadcaster_->sendTransform(odom_to_base);

    cmd_vel_sub_ = client_node_->create_subscription<geometry_msgs::msg::TwistStamped>(
      "cmd_vel", 100, [this](geometry_msgs::msg::TwistStamped::ConstSharedPtr msg) {
        std::lock_guard<std::mutex> lock(cmd_mutex_);
        cmd_vels_.push_back(msg->twist);
      });
    action_client_ = rclcpp_action::create_client<FollowPath>(client_node_, "follow_path");

    server_ = std::make_shared<TestControllerServer>();
    server_->configure();
    controller_ = std::make_shared<RecordingController>();
    server_->replaceController("FollowPath", controller_);
    server_->activate();

    executor_.add_node(server_->get_node_base_interface());
    executor_.add_node(client_node_);
    spin_thread_ = std::thread([this]() {executor_.spin();});
    ASSERT_TRUE(action_client_->wait_for_action_server(10s));
  }

  void TearDown() override
  {
    if (!server_) {
      return;
    }
    executor_.cancel();
    spin_thread_.join();
    server_->deactivate();
    server_->cleanup();
    server_.reset();
  }

  FollowPath::Goal goalTo(double x)
  {
    FollowPath::Goal goal;
    goal.path.header.frame_id = "odom";
    goal.path.header.stamp = client_node_->now();
    goal.path.poses.push_back(odomPose(x));
    goal.controller_id = "FollowPath";
    return goal;
  }

  std::vector<geometry_msgs::msg::Twist> cmdVels()
  {
    std::lock_guard<std::mutex> lock(cmd_mutex_);
    return cmd_vels_;
  }

  rclcpp::Node::SharedPtr client_node_;
  std::shared_ptr<tf2_ros::StaticTransformBroadcaster> tf_broadcaster_;
  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr cmd_vel_sub_;
  rclcpp_action::Client<FollowPath>::SharedPtr action_client_;
  std::shared_ptr<TestControllerServer> server_;
  std::shared_ptr<RecordingController> controller_;
  rclcpp::executors::MultiThreadedExecutor executor_;
  std::thread spin_thread_;
  std::mutex cmd_mutex_;
  std::vector<geometry_msgs::msg::Twist> cmd_vels_;
};

TEST_F(GoalReachedBeforeControlTest, PathOfOnlyTheOwnPoseGetsNoControlStep)
{
  // The BT re-sends the path that is only the robot's own pose: every goal ends as reached
  // without the controller being asked for a command, and only the stop goes out
  for (int i = 0; i < 5; ++i) {
    auto goal_handle_future = action_client_->async_send_goal(goalTo(0.0));
    ASSERT_EQ(goal_handle_future.wait_for(5s), std::future_status::ready);
    auto goal_handle = goal_handle_future.get();
    ASSERT_NE(goal_handle, nullptr);
    auto result_future = action_client_->async_get_result(goal_handle);
    ASSERT_EQ(result_future.wait_for(5s), std::future_status::ready);
    EXPECT_EQ(result_future.get().code, rclcpp_action::ResultCode::SUCCEEDED);
  }
  std::this_thread::sleep_for(200ms);

  EXPECT_EQ(controller_->calls.load(), 0);
  const auto cmds = cmdVels();
  EXPECT_FALSE(cmds.empty());
  for (const auto & cmd : cmds) {
    EXPECT_EQ(cmd.linear.x, 0.0);
    EXPECT_EQ(cmd.angular.z, 0.0);
  }
}

TEST_F(GoalReachedBeforeControlTest, GoalAheadIsStillControlled)
{
  // A goal 1 m ahead (inside the 4 m rolling costmap) is driven as before: the controller's
  // command goes out
  auto goal_handle_future = action_client_->async_send_goal(goalTo(1.0));
  ASSERT_EQ(goal_handle_future.wait_for(5s), std::future_status::ready);
  auto goal_handle = goal_handle_future.get();
  ASSERT_NE(goal_handle, nullptr);

  const auto deadline = std::chrono::steady_clock::now() + 5s;
  bool command_seen = false;
  while (!command_seen && std::chrono::steady_clock::now() < deadline) {
    for (const auto & cmd : cmdVels()) {
      command_seen = command_seen || cmd.linear.x == 0.01;
    }
    std::this_thread::sleep_for(20ms);
  }
  EXPECT_TRUE(command_seen);
  EXPECT_GT(controller_->calls.load(), 0);

  auto cancel_future = action_client_->async_cancel_goal(goal_handle);
  EXPECT_EQ(cancel_future.wait_for(5s), std::future_status::ready);
}

}  // namespace

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);

  // One parameter file for the controller server and its two costmaps: a small rolling
  // costmap in odom with only an inflation layer, and the defaults of the server otherwise
  const std::string params_file = "/tmp/test_goal_reached_before_control_" +
    std::to_string(::getpid()) + ".yaml";
  {
    std::ofstream out(params_file);
    out << "/**:\n"
      "  ros__parameters:\n"
      "    global_frame: odom\n"
      "    robot_base_frame: base_link\n"
      "    rolling_window: true\n"
      "    width: 4\n"
      "    height: 4\n"
      "    resolution: 0.05\n"
      "    update_frequency: 10.0\n"
      "    publish_frequency: 1.0\n"
      "    plugins: [\"inflation_layer\"]\n"
      "    inflation_layer:\n"
      "      plugin: \"nav2_costmap_2d::InflationLayer\"\n"
      "    enable_stamped_cmd_vel: true\n"
      "    controller_frequency: 20.0\n"
      "    controller_plugins: [\"FollowPath\"]\n"
      "    FollowPath:\n"
      "      plugin: \"nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController\"\n";
  }
  const char * ros_args[] = {"test", "--ros-args", "--params-file", params_file.c_str()};
  rclcpp::init(4, ros_args);

  int result = RUN_ALL_TESTS();

  rclcpp::shutdown();
  std::remove(params_file.c_str());
  return result;
}
