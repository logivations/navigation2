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

#ifndef NAV2_BEHAVIOR_TREE__BT_SHARED_INTERFACES_HPP_
#define NAV2_BEHAVIOR_TREE__BT_SHARED_INTERFACES_HPP_

#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <utility>

#include "rclcpp/rclcpp.hpp"

namespace nav2_behavior_tree
{

namespace detail
{

/// The shared T's of all ROS nodes, by node and name. One per T, whatever the call site.
template<typename T>
struct SharedInterfaceRegistry
{
  struct Entry
  {
    std::weak_ptr<rclcpp::node_interfaces::NodeBaseInterface> node;
    std::weak_ptr<T> value;
  };

  static SharedInterfaceRegistry & instance()
  {
    static SharedInterfaceRegistry registry;
    return registry;
  }

  std::mutex mutex;
  std::map<std::pair<const void *, std::string>, Entry> entries;
};

}  // namespace detail

/**
 * @brief Returns the T that the BT nodes of @p node share under @p name, creating it with
 * @p make on first use.
 *
 * A behavior tree holds many nodes that talk to the same server or topic (dozens of Wait or
 * Spin nodes in one tree are common). Giving each its own client or publisher creates thousands
 * of DDS endpoints per navigator, and building a tree then takes seconds on a busy robot. The
 * BT nodes of one ROS node share one instance per name instead. An entry lives as long as a BT
 * node holds it and is recreated after the last holder (or the ROS node) is gone.
 *
 * Anything with an executor in it may only be spun from one thread. BtActionServer ticks all
 * trees of a navigator from its one action server thread, and each navigator has its own node.
 */
template<typename T, typename Factory>
std::shared_ptr<T> getSharedInterface(
  const rclcpp::node_interfaces::NodeBaseInterface::SharedPtr & node,
  const std::string & name, Factory && make)
{
  auto & registry = detail::SharedInterfaceRegistry<T>::instance();
  std::lock_guard<std::mutex> lock(registry.mutex);
  auto & entry = registry.entries[{node.get(), name}];
  auto value = entry.value.lock();
  // A new node at the address of a destroyed one must not get its leftovers
  if (!value || entry.node.expired()) {
    value = make();
    entry = {node, value};
  }
  return value;
}

}  // namespace nav2_behavior_tree

#endif  // NAV2_BEHAVIOR_TREE__BT_SHARED_INTERFACES_HPP_
