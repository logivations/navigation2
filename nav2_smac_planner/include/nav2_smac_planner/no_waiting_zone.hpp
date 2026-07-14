// Copyright (c) 2026, Pixel Robotics GmbH
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
// limitations under the License. Reserved.

#ifndef NAV2_SMAC_PLANNER__NO_WAITING_ZONE_HPP_
#define NAV2_SMAC_PLANNER__NO_WAITING_ZONE_HPP_

#include <cmath>
#include <functional>
#include <memory>
#include <mutex>
#include <string>

#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav2_core/planner_exceptions.hpp"
#include "nav2_costmap_2d/costmap_2d.hpp"
#include "nav2_ros_common/lifecycle_node.hpp"
#include "nav2_ros_common/node_utils.hpp"
#include "nav2_ros_common/qos_profiles.hpp"
#include "nav2_ros_common/subscription.hpp"
#include "tf2/LinearMath/Quaternion.hpp"
#include "tf2/utils.hpp"

namespace nav2_smac_planner
{

/**
 * @class nav2_smac_planner::NoWaitingZone
 * @brief Subscribes to an OccupancyGrid topic describing a "no-waiting zone"
 * and provides stop checkers for the free space search ("find free space"
 * mode) of the A* algorithm. The zone cannot be marked in the global costmap
 * since the robot may start inside of it — it only constrains where the
 * search is allowed to stop, not where it may travel through.
 */
class NoWaitingZone
{
public:
  /**
   * @brief Declare parameters and subscribe to the no-waiting zone topic
   * @param node Lifecycle node to declare parameters on and subscribe with
   * @param plugin_name Name of the planner plugin, used as parameter namespace
   */
  void configure(const nav2::LifecycleNode::SharedPtr & node, const std::string & plugin_name)
  {
    _topic = node->declare_or_get_parameter(
      plugin_name + ".no_waiting_zone_topic", std::string("no_waiting_zone"));
    _occupied_threshold = node->declare_or_get_parameter(
      plugin_name + ".no_waiting_zone_occupied_threshold", 1);
    _grid_sub = node->create_subscription<nav_msgs::msg::OccupancyGrid>(
      _topic,
      [this](nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(_mutex);
        _grid = msg;
      },
      nav2::qos::LatchedSubscriptionQoS());
  }

  /**
   * @brief Release the subscription and any stored grid
   */
  void cleanup()
  {
    _grid_sub.reset();
    std::lock_guard<std::mutex> lock(_mutex);
    _grid.reset();
  }

  /**
   * @brief Get the topic the zone is subscribed on
   * @return Topic name
   */
  const std::string & getTopic() const
  {
    return _topic;
  }

  /**
   * @brief Create a stop checker functor for AStarAlgorithm::enableFreeSpaceSearch
   * bound to a snapshot of the last received zone grid. The functor converts
   * costmap cell coordinates to world coordinates and returns true if the
   * position is outside of the no-waiting zone, i.e. a valid position to stop.
   * @param costmap Costmap the search runs on (used for cell-to-world conversion)
   * @param global_frame Global frame of the planner, to validate the zone frame
   * @return Stop checker functor
   */
  std::function<bool(const float &, const float &)> createFreeSpaceStopChecker(
    const nav2_costmap_2d::Costmap2D * costmap, const std::string & global_frame)
  {
    nav_msgs::msg::OccupancyGrid::ConstSharedPtr grid;
    {
      std::lock_guard<std::mutex> lock(_mutex);
      grid = _grid;
    }

    if (!grid) {
      throw nav2_core::PlannerException(
              "Find free space mode is active but no no-waiting zone grid has been "
              "received yet on topic: " + _topic);
    }

    if (!grid->header.frame_id.empty() && grid->header.frame_id != global_frame) {
      throw nav2_core::PlannerException(
              "No-waiting zone grid frame '" + grid->header.frame_id +
              "' does not match the planner frame '" + global_frame + "'");
    }

    const int8_t threshold = static_cast<int8_t>(_occupied_threshold);
    const double resolution = costmap->getResolution();
    const double origin_x = costmap->getOriginX();
    const double origin_y = costmap->getOriginY();
    return [grid, origin_x, origin_y, resolution, threshold](
      const float & mx, const float & my) -> bool {
        const double wx = origin_x + (static_cast<double>(mx) + 0.5) * resolution;
        const double wy = origin_y + (static_cast<double>(my) + 0.5) * resolution;
        return !isInZone(*grid, wx, wy, threshold);
      };
  }

  /**
   * @brief Check if a world position lies within the no-waiting zone.
   * Positions outside the grid bounds and cells with unknown (< 0) occupancy
   * are considered outside of the zone.
   * @param grid Zone occupancy grid
   * @param wx World X coordinate
   * @param wy World Y coordinate
   * @param occupied_threshold Minimum occupancy value considered inside the zone
   * @return If the position is inside the no-waiting zone
   */
  static bool isInZone(
    const nav_msgs::msg::OccupancyGrid & grid,
    const double & wx, const double & wy,
    const int8_t & occupied_threshold)
  {
    // Transform the world position into the (possibly rotated) grid frame
    const double dx = wx - grid.info.origin.position.x;
    const double dy = wy - grid.info.origin.position.y;
    const auto & o = grid.info.origin.orientation;
    const double yaw = tf2::getYaw(tf2::Quaternion(o.x, o.y, o.z, o.w));
    const double gx = std::cos(yaw) * dx + std::sin(yaw) * dy;
    const double gy = -std::sin(yaw) * dx + std::cos(yaw) * dy;

    if (gx < 0.0 || gy < 0.0) {
      return false;
    }

    const auto mx = static_cast<unsigned int>(gx / grid.info.resolution);
    const auto my = static_cast<unsigned int>(gy / grid.info.resolution);
    if (mx >= grid.info.width || my >= grid.info.height) {
      return false;
    }

    const int8_t value = grid.data[my * grid.info.width + mx];
    return value >= occupied_threshold;
  }

protected:
  std::string _topic;
  int _occupied_threshold{1};
  nav2::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr _grid_sub;
  nav_msgs::msg::OccupancyGrid::ConstSharedPtr _grid;
  std::mutex _mutex;
};

}  // namespace nav2_smac_planner

#endif  // NAV2_SMAC_PLANNER__NO_WAITING_ZONE_HPP_
