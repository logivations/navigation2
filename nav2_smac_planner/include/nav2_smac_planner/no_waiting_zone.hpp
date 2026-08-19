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

#include <algorithm>
#include <cmath>
#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include "geometry_msgs/msg/point.hpp"
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
 * @enum nav2_smac_planner::FreeSpaceHeadingMode
 * @brief How strongly the free space search constrains the heading of the pose
 * it stops at: not at all, prefer an aligned one but settle for any legal one,
 * or only ever stop at an aligned one
 */
enum class FreeSpaceHeadingMode
{
  UNKNOWN = 0,
  NONE = 1,
  PREFERRED = 2,
  FORCED = 3
};

/**
 * @brief Parse a free space heading mode parameter value
 * @param mode Parameter value, one of "none", "preferred", "forced"
 * @return The mode, or UNKNOWN if the string matches none of them
 */
inline FreeSpaceHeadingMode fromStringToFSH(const std::string & mode)
{
  if (mode == "none") {
    return FreeSpaceHeadingMode::NONE;
  } else if (mode == "preferred") {
    return FreeSpaceHeadingMode::PREFERRED;
  } else if (mode == "forced") {
    return FreeSpaceHeadingMode::FORCED;
  }
  return FreeSpaceHeadingMode::UNKNOWN;
}

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
    _padding = node->declare_or_get_parameter(
      plugin_name + ".no_waiting_zone_padding", 0.0);
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
  std::function<bool(const float &, const float &, const float &)> createFreeSpaceStopChecker(
    const nav2_costmap_2d::Costmap2D * costmap, const std::string & global_frame,
    const std::vector<geometry_msgs::msg::Point> & footprint = {})
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
    const double padding = _padding;
    const double resolution = costmap->getResolution();
    const double origin_x = costmap->getOriginX();
    const double origin_y = costmap->getOriginY();
    return [grid, origin_x, origin_y, resolution, threshold, padding, footprint](
      const float & mx, const float & my, const float & theta) -> bool {
        const double wx = origin_x + (static_cast<double>(mx) + 0.5) * resolution;
        const double wy = origin_y + (static_cast<double>(my) + 0.5) * resolution;
        return !footprintInZone(*grid, wx, wy, theta, footprint, threshold, padding);
      };
  }

  /**
   * @brief Check if the robot footprint, placed at a pose, overlaps the no-waiting zone.
   * A single point cannot describe an elongated robot: an isotropic padding large enough to
   * cover it inflates the zone by the circumscribed radius in every direction, which can leave
   * no reachable stop position at all in a narrow corridor. The zone is a filled region, so
   * sampling the footprint outline at grid resolution finds every overlap a fill test would;
   * the centre is tested too for the degenerate case of a zone smaller than the robot.
   * @param grid Zone occupancy grid
   * @param wx World X of the robot origin
   * @param wy World Y of the robot origin
   * @param theta Robot heading in radians
   * @param footprint Robot footprint in the robot frame; empty falls back to a point check
   * @param occupied_threshold Minimum occupancy value considered inside the zone
   * @param padding Minimum clearance to the zone in meters (<= 0 disables)
   * @return If any part of the footprint lies inside the (padded) no-waiting zone
   */
  static bool footprintInZone(
    const nav_msgs::msg::OccupancyGrid & grid,
    const double & wx, const double & wy, const double & theta,
    const std::vector<geometry_msgs::msg::Point> & footprint,
    const int8_t & occupied_threshold,
    const double & padding = 0.0)
  {
    if (isInZone(grid, wx, wy, occupied_threshold, padding)) {
      return true;
    }
    if (footprint.size() < 3) {
      return false;
    }

    const double cs = std::cos(theta);
    const double sn = std::sin(theta);
    const double step = std::max(grid.info.resolution * 0.5, 1e-3);

    for (size_t i = 0; i < footprint.size(); ++i) {
      const auto & p0 = footprint[i];
      const auto & p1 = footprint[(i + 1) % footprint.size()];
      const double ax = wx + p0.x * cs - p0.y * sn;
      const double ay = wy + p0.x * sn + p0.y * cs;
      const double bx = wx + p1.x * cs - p1.y * sn;
      const double by = wy + p1.x * sn + p1.y * cs;
      const int samples = std::max(1, static_cast<int>(std::ceil(std::hypot(bx - ax, by - ay) /
        step)));
      for (int k = 0; k <= samples; ++k) {
        const double t = static_cast<double>(k) / static_cast<double>(samples);
        if (isInZone(grid, ax + (bx - ax) * t, ay + (by - ay) * t, occupied_threshold, padding)) {
          return true;
        }
      }
    }
    return false;
  }

  /**
   * @brief Check if a world position lies within the no-waiting zone.
   * Positions outside the grid bounds and cells with unknown (< 0) occupancy
   * are considered outside of the zone. With a positive padding, positions
   * closer than `padding` to any zone cell also count as inside, so a stop
   * position keeps at least that distance to every zone (e.g. so the robot
   * footprint does not hang into the zone, and the path end lies farther
   * beyond the zone edge than the follower's goal tolerance).
   * @param grid Zone occupancy grid
   * @param wx World X coordinate
   * @param wy World Y coordinate
   * @param occupied_threshold Minimum occupancy value considered inside the zone
   * @param padding Minimum clearance to the zone in meters (<= 0 disables)
   * @return If the position is inside the (padded) no-waiting zone
   */
  static bool isInZone(
    const nav_msgs::msg::OccupancyGrid & grid,
    const double & wx, const double & wy,
    const int8_t & occupied_threshold,
    const double & padding = 0.0)
  {
    // Transform the world position into the (possibly rotated) grid frame
    const double dx = wx - grid.info.origin.position.x;
    const double dy = wy - grid.info.origin.position.y;
    const auto & o = grid.info.origin.orientation;
    const double yaw = tf2::getYaw(tf2::Quaternion(o.x, o.y, o.z, o.w));
    const double gx = std::cos(yaw) * dx + std::sin(yaw) * dy;
    const double gy = -std::sin(yaw) * dx + std::cos(yaw) * dy;
    const double res = grid.info.resolution;

    if (padding <= 0.0) {
      if (gx < 0.0 || gy < 0.0) {
        return false;
      }
      const auto mx = static_cast<unsigned int>(gx / res);
      const auto my = static_cast<unsigned int>(gy / res);
      if (mx >= grid.info.width || my >= grid.info.height) {
        return false;
      }
      return grid.data[my * grid.info.width + mx] >= occupied_threshold;
    }

    // Padded check: scan the zone cells in the padding-radius box around the
    // position and compare against the nearest point of each occupied cell.
    // Rotation preserves distances, so the radius is the same in grid frame.
    const int r = static_cast<int>(std::ceil(padding / res));
    const int cx = static_cast<int>(std::floor(gx / res));
    const int cy = static_cast<int>(std::floor(gy / res));
    const double pad_sq = padding * padding;
    for (int iy = std::max(cy - r, 0);
      iy <= std::min(cy + r, static_cast<int>(grid.info.height) - 1); ++iy)
    {
      for (int ix = std::max(cx - r, 0);
        ix <= std::min(cx + r, static_cast<int>(grid.info.width) - 1); ++ix)
      {
        if (grid.data[iy * grid.info.width + ix] < occupied_threshold) {
          continue;
        }
        const double ddx = std::max({ix * res - gx, 0.0, gx - (ix + 1) * res});
        const double ddy = std::max({iy * res - gy, 0.0, gy - (iy + 1) * res});
        if (ddx * ddx + ddy * ddy <= pad_sq) {
          return true;
        }
      }
    }
    return false;
  }

protected:
  std::string _topic;
  int _occupied_threshold{1};
  double _padding{0.0};
  nav2::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr _grid_sub;
  nav_msgs::msg::OccupancyGrid::ConstSharedPtr _grid;
  std::mutex _mutex;
};

}  // namespace nav2_smac_planner

#endif  // NAV2_SMAC_PLANNER__NO_WAITING_ZONE_HPP_
