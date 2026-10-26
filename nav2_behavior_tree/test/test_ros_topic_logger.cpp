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

#include <gtest/gtest.h>

#include <memory>
#include <thread>

#include "behaviortree_cpp/bt_factory.h"
#include "nav2_behavior_tree/ros_topic_logger.hpp"
#include "nav2_ros_common/lifecycle_node.hpp"

class TestRosTopicLogger : public nav2_behavior_tree::RosTopicLogger
{
public:
  using RosTopicLogger::RosTopicLogger;
  size_t pending() const {return event_log_.size();}
};

// BT::TimeoutNode halts its child on its own timer thread, so callback() also runs there while
// the BT thread logs its own transitions and flushes. Unsynchronized, this corrupted the event
// vector and the heap (AMRNAV-8396: bt_navigator froze on an AMR).
TEST(RosTopicLoggerTest, test_callbacks_from_timer_thread)
{
  auto node = std::make_shared<nav2::LifecycleNode>("ros_topic_logger_test");
  BT::BehaviorTreeFactory factory;
  auto tree = factory.createTreeFromText(
    R"(<root BTCPP_format="4"><BehaviorTree ID="Main"><AlwaysSuccess/></BehaviorTree></root>)");
  TestRosTopicLogger logger(node, tree);
  const BT::TreeNode & bt_node = *tree.rootNode();
  constexpr size_t kEvents = 20000;
  auto log_events = [&]() {
      for (size_t i = 0; i < kEvents; ++i) {
        logger.callback(BT::Duration{}, bt_node, BT::NodeStatus::RUNNING, BT::NodeStatus::IDLE);
      }
    };

  std::thread timer_thread(log_events);
  log_events();
  timer_thread.join();
  EXPECT_EQ(logger.pending(), 2 * kEvents);

  std::thread timer_thread_during_flush(log_events);
  for (size_t i = 0; i < kEvents; ++i) {
    logger.flush();
  }
  timer_thread_during_flush.join();
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  int all_successful = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return all_successful;
}
