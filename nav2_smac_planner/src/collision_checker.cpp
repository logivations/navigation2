// Copyright (c) 2021, Samsung Research America
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

#include "nav2_smac_planner/collision_checker.hpp"

#include <algorithm>
#include <cmath>

#include "nav2_smac_planner/utils.hpp"

namespace nav2_smac_planner
{

GridCollisionChecker::GridCollisionChecker(
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros,
  unsigned int num_quantizations,
  nav2::LifecycleNode::SharedPtr node)
: FootprintCollisionChecker(costmap_ros ? costmap_ros->getCostmap() : nullptr)
{
  if (node) {
    clock_ = node->get_clock();
    logger_ = node->get_logger();
  }

  if (costmap_ros) {
    costmap_ros_ = costmap_ros;
  }

  // Convert number of regular bins into angles
  float bin_size = 2 * M_PI / static_cast<float>(num_quantizations);
  angles_.reserve(num_quantizations);
  for (unsigned int i = 0; i != num_quantizations; i++) {
    angles_.push_back(bin_size * i);
  }
}

// GridCollisionChecker::GridCollisionChecker(
//   nav2_costmap_2d::Costmap2D * costmap,
//   std::vector<float> & angles)
// : FootprintCollisionChecker(costmap),
//   angles_(angles)
// {
// }

void GridCollisionChecker::setFootprint(
  const nav2_costmap_2d::Footprint & footprint,
  const bool & radius,
  const double & possible_collision_cost)
{
  possible_collision_cost_ = static_cast<float>(possible_collision_cost);
  if (possible_collision_cost_ <= 0.0f) {
    RCLCPP_ERROR_THROTTLE(
      logger_, *clock_, 1000,
      "Inflation layer either not found or inflation is not set sufficiently for "
      "optimized non-circular collision checking capabilities. It is HIGHLY recommended to set"
      " the inflation radius to be at MINIMUM half of the robot's largest cross-section. See "
      "github.com/ros-planning/navigation2/tree/main/nav2_smac_planner#potential-fields"
      " for full instructions. This will substantially impact run-time performance.");
  }

  footprint_is_radius_ = radius;

  // Use radius, no caching required
  if (radius) {
    return;
  }

  // No change, no updates required
  if (footprint == unoriented_footprint_) {
    return;
  }
  unoriented_footprint_ = footprint;
  updateSoftFootprint();

  oriented_footprints_.clear();
  oriented_footprints_.reserve(angles_.size());
  double sin_th, cos_th;
  geometry_msgs::msg::Point new_pt;
  const unsigned int footprint_size = footprint.size();

  // Precompute the orientation bins for checking to use
  for (unsigned int i = 0; i != angles_.size(); i++) {
    sin_th = sin(angles_[i]);
    cos_th = cos(angles_[i]);
    nav2_costmap_2d::Footprint oriented_footprint;
    oriented_footprint.reserve(footprint_size);

    for (unsigned int j = 0; j < footprint_size; j++) {
      new_pt.x = footprint[j].x * cos_th - footprint[j].y * sin_th;
      new_pt.y = footprint[j].x * sin_th + footprint[j].y * cos_th;
      oriented_footprint.push_back(new_pt);
    }

    oriented_footprints_.push_back(oriented_footprint);
  }
}

void GridCollisionChecker::setSoftFootprintPadding(
  const float front, const float rear, const float side)
{
  soft_padding_front_ = std::max(front, 0.0f);
  soft_padding_rear_ = std::max(rear, 0.0f);
  soft_padding_side_ = std::max(side, 0.0f);
  updateSoftFootprint();
}

void GridCollisionChecker::updateSoftFootprint()
{
  oriented_padded_footprints_.clear();
  soft_padding_enabled_ = !unoriented_footprint_.empty() &&
    (soft_padding_front_ > 0.0f || soft_padding_rear_ > 0.0f || soft_padding_side_ > 0.0f);
  if (!soft_padding_enabled_) {
    return;
  }

  // grow every vertex away from base_link along x (front / rear) and y (sides)
  nav2_costmap_2d::Footprint padded = unoriented_footprint_;
  double circumscribed_radius = 0.0;
  for (auto & pt : padded) {
    pt.x += pt.x > 0.0 ? soft_padding_front_ : -soft_padding_rear_;
    pt.y += pt.y > 0.0 ? soft_padding_side_ : (pt.y < 0.0 ? -soft_padding_side_ : 0.0);
    circumscribed_radius = std::max(circumscribed_radius, std::hypot(pt.x, pt.y));
  }

  // below this center cost, no lethal cell can be within the padded footprint
  possible_soft_collision_cost_ = costmap_ros_ ?
    static_cast<float>(findCostAtRadius(costmap_ros_, circumscribed_radius)) : -1.0f;

  geometry_msgs::msg::Point new_pt;
  oriented_padded_footprints_.reserve(angles_.size());
  for (unsigned int i = 0; i != angles_.size(); i++) {
    const double sin_th = sin(angles_[i]);
    const double cos_th = cos(angles_[i]);
    nav2_costmap_2d::Footprint oriented;
    oriented.reserve(padded.size());
    for (const auto & pt : padded) {
      new_pt.x = pt.x * cos_th - pt.y * sin_th;
      new_pt.y = pt.x * sin_th + pt.y * cos_th;
      oriented.push_back(new_pt);
    }
    oriented_padded_footprints_.push_back(oriented);
  }
}

bool GridCollisionChecker::softFootprintHitsLethal(
  const double & wx, const double & wy, const float & angle_bin)
{
  const nav2_costmap_2d::Footprint & oriented =
    oriented_padded_footprints_[static_cast<unsigned int>(angle_bin)];
  nav2_costmap_2d::Footprint current;
  current.reserve(oriented.size());
  geometry_msgs::msg::Point new_pt;
  for (const auto & pt : oriented) {
    new_pt.x = wx + pt.x;
    new_pt.y = wy + pt.y;
    current.push_back(new_pt);
  }
  const float cost = static_cast<float>(footprintCost(current));
  return cost >= OCCUPIED_COST && cost != UNKNOWN_COST;
}

bool GridCollisionChecker::inCollision(
  const float & x,
  const float & y,
  const float & angle_bin,
  const bool & traverse_unknown)
{
  // Check to make sure cell is inside the map
  if (outsideRange(costmap_->getSizeInCellsX(), x) ||
    outsideRange(costmap_->getSizeInCellsY(), y))
  {
    return true;
  }

  // Assumes setFootprint already set
  center_cost_ = static_cast<float>(costmap_->getCost(
      static_cast<unsigned int>(x + 0.5f), static_cast<unsigned int>(y + 0.5f)));
  soft_violation_ = false;

  if (!footprint_is_radius_) {
    // the soft-padded footprint can only touch a lethal cell above its shortcut cost
    const bool check_soft = soft_padding_enabled_ &&
      !(center_cost_ < possible_soft_collision_cost_ && possible_soft_collision_cost_ > 0.0f);

    // if footprint, then we check for the footprint's points, but first see
    // if the robot is even potentially in an inscribed collision
    if (center_cost_ < possible_collision_cost_ && possible_collision_cost_ > 0.0f) {
      if (check_soft) {
        double wx, wy;
        costmap_->mapToWorld(static_cast<double>(x), static_cast<double>(y), wx, wy);
        soft_violation_ = softFootprintHitsLethal(wx, wy, angle_bin);
      }
      return false;
    }

    // If its inscribed, in collision, or unknown in the middle,
    // no need to even check the footprint, its invalid
    if (center_cost_ == UNKNOWN_COST && !traverse_unknown) {
      return true;
    }

    if (center_cost_ == INSCRIBED_COST || center_cost_ == OCCUPIED_COST) {
      return true;
    }

    // if possible inscribed, need to check actual footprint pose.
    // Use precomputed oriented footprints are done on initialization,
    // offset by translation value to collision check
    double wx, wy;
    costmap_->mapToWorld(static_cast<double>(x), static_cast<double>(y), wx, wy);
    geometry_msgs::msg::Point new_pt;
    const nav2_costmap_2d::Footprint & oriented_footprint = oriented_footprints_[angle_bin];
    nav2_costmap_2d::Footprint current_footprint;
    current_footprint.reserve(oriented_footprint.size());
    for (unsigned int i = 0; i < oriented_footprint.size(); ++i) {
      new_pt.x = wx + oriented_footprint[i].x;
      new_pt.y = wy + oriented_footprint[i].y;
      current_footprint.push_back(new_pt);
    }

    float footprint_cost = static_cast<float>(footprintCost(current_footprint));

    if (footprint_cost == UNKNOWN_COST && traverse_unknown) {
      return false;
    }

    // if occupied or unknown and not to traverse unknown space
    const bool collision = footprint_cost >= OCCUPIED_COST;
    if (!collision && check_soft) {
      soft_violation_ = softFootprintHitsLethal(wx, wy, angle_bin);
    }
    return collision;
  } else {
    // if radius, then we can check the center of the cost assuming inflation is used
    if (center_cost_ == UNKNOWN_COST && traverse_unknown) {
      return false;
    }

    // if occupied or unknown and not to traverse unknown space
    return center_cost_ >= INSCRIBED_COST;
  }
}

bool GridCollisionChecker::inCollision(
  const unsigned int & i,
  const bool & traverse_unknown)
{
  center_cost_ = costmap_->getCost(i);
  soft_violation_ = false;
  if (center_cost_ == UNKNOWN_COST && traverse_unknown) {
    return false;
  }

  // if occupied or unknown and not to traverse unknown space
  return center_cost_ >= INSCRIBED_COST;
}

float GridCollisionChecker::getCost()
{
  // Assumes inCollision called prior
  return static_cast<float>(center_cost_);
}

bool GridCollisionChecker::outsideRange(const unsigned int & max, const float & value)
{
  return value < 0.0f || value > max;
}

}  // namespace nav2_smac_planner
