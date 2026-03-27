// Copyright (c) 2024 Open Navigation LLC
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

#ifndef NAV2_BEHAVIOR_TREE__ROS_TRANSITION_LOGGER_HPP_
#define NAV2_BEHAVIOR_TREE__ROS_TRANSITION_LOGGER_HPP_

#include <memory>
#include <string>

#include "behaviortree_cpp/loggers/abstract_logger.h"
#include "rclcpp/rclcpp.hpp"
#include "nav2_msgs/msg/behavior_tree_transition.hpp"
#include "nav2_msgs/msg/behavior_tree_snapshot.hpp"
#include "nav2_msgs/msg/behavior_tree_node_state.hpp"
#include "tf2/time.hpp"
#include "tf2_ros/buffer_interface.hpp"

namespace nav2_behavior_tree
{

/**
 * @brief A logger that publishes individual BT transitions and periodic snapshots.
 *
 * Unlike RosTopicLogger which batches transitions, this logger:
 * - Publishes each status change as an individual BehaviorTreeTransition message
 * - Periodically publishes a BehaviorTreeSnapshot with full tree state + uid-to-name mapping
 *
 * Designed for mcap-to-database pipelines where each transition should be a queryable row.
 */
class RosTransitionLogger : public BT::StatusChangeLogger
{
public:
  /**
   * @brief Constructor
   * @param ros_node Weak pointer to parent LifecycleNode
   * @param tree BT to monitor
   * @param log_idle Whether to log transitions to IDLE state
   * @param snapshot_interval Seconds between full tree snapshots (default 5.0)
   */
  RosTransitionLogger(
    const nav2::LifecycleNode::WeakPtr & ros_node,
    const BT::Tree & tree,
    bool log_idle = true,
    double snapshot_interval = 10.0)
  : StatusChangeLogger(tree.rootNode()),
    tree_(tree),
    snapshot_interval_(snapshot_interval)
  {
    auto node = ros_node.lock();
    clock_ = node->get_clock();

    transition_pub_ = node->create_publisher<nav2_msgs::msg::BehaviorTreeTransition>(
      "~/bt_transition",
      rclcpp::SensorDataQoS());

    snapshot_pub_ = node->create_publisher<nav2_msgs::msg::BehaviorTreeSnapshot>(
      "~/bt_snapshot",
      rclcpp::QoS(1).transient_local());

    enableTransitionToIdle(log_idle);

    // Build the tree_id from the first subtree
    if (!tree_.subtrees.empty()) {
      tree_id_ = tree_.subtrees.front()->tree_ID;
    }

    // Publish initial snapshot immediately
    last_snapshot_time_ = clock_->now();
    publishSnapshot();
  }

  /**
   * @brief Callback for each BT node status change — publishes immediately
   */
  void callback(
    BT::Duration timestamp,
    const BT::TreeNode & node,
    BT::NodeStatus prev_status,
    BT::NodeStatus status) override
  {
    auto msg = std::make_unique<nav2_msgs::msg::BehaviorTreeTransition>();
    msg->timestamp = tf2_ros::toMsg(tf2::TimePoint(timestamp));
    msg->node_uid = static_cast<uint16_t>(node.UID());
    msg->previous_status = toStatusUint8(prev_status);
    msg->current_status = toStatusUint8(status);
    transition_pub_->publish(std::move(msg));
  }

  /**
   * @brief Called each tick — publishes snapshot if interval has elapsed
   */
  void flush() override
  {
    auto now = clock_->now();
    if ((now - last_snapshot_time_).seconds() >= snapshot_interval_) {
      last_snapshot_time_ = now;
      publishSnapshot();
    }
  }

private:
  /**
   * @brief Convert BT::NodeStatus enum to uint8 message constant
   */
  static uint8_t toStatusUint8(BT::NodeStatus status)
  {
    // BT::NodeStatus enum values match our message constants exactly
    return static_cast<uint8_t>(status);
  }

  /**
   * @brief Convert BT::NodeType enum to string
   */
  static std::string nodeTypeToStr(BT::NodeType type)
  {
    switch (type) {
      case BT::NodeType::ACTION: return "Action";
      case BT::NodeType::CONDITION: return "Condition";
      case BT::NodeType::CONTROL: return "Control";
      case BT::NodeType::DECORATOR: return "Decorator";
      case BT::NodeType::SUBTREE: return "SubTree";
      default: return "Undefined";
    }
  }

  /**
   * @brief Publish a full snapshot of the tree with uid→name mapping + current status
   */
  void publishSnapshot()
  {
    auto msg = std::make_unique<nav2_msgs::msg::BehaviorTreeSnapshot>();
    msg->timestamp = clock_->now();
    msg->tree_id = tree_id_;

    for (const auto & subtree : tree_.subtrees) {
      for (const auto & node : subtree->nodes) {
        nav2_msgs::msg::BehaviorTreeNodeState state;
        state.node_uid = static_cast<uint16_t>(node->UID());
        state.node_name = node->name();
        state.node_type = nodeTypeToStr(node->type());
        state.status = toStatusUint8(node->status());
        msg->nodes.push_back(std::move(state));
      }
    }

    snapshot_pub_->publish(std::move(msg));
  }

  const BT::Tree & tree_;
  std::string tree_id_;
  double snapshot_interval_;
  rclcpp::Time last_snapshot_time_;
  rclcpp::Clock::SharedPtr clock_;
  rclcpp::Publisher<nav2_msgs::msg::BehaviorTreeTransition>::SharedPtr transition_pub_;
  rclcpp::Publisher<nav2_msgs::msg::BehaviorTreeSnapshot>::SharedPtr snapshot_pub_;
};

}   // namespace nav2_behavior_tree

#endif   // NAV2_BEHAVIOR_TREE__ROS_TRANSITION_LOGGER_HPP_
