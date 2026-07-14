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

#include <cmath>
#include <memory>

#include "gtest/gtest.h"
#include "nav2_costmap_2d/costmap_2d.hpp"
#include "nav2_smac_planner/no_waiting_zone.hpp"

using nav2_smac_planner::NoWaitingZone;

nav_msgs::msg::OccupancyGrid makeGrid(
  const unsigned int width, const unsigned int height, const double resolution,
  const double origin_x, const double origin_y, const double yaw = 0.0)
{
  nav_msgs::msg::OccupancyGrid grid;
  grid.header.frame_id = "map";
  grid.info.width = width;
  grid.info.height = height;
  grid.info.resolution = resolution;
  grid.info.origin.position.x = origin_x;
  grid.info.origin.position.y = origin_y;
  grid.info.origin.orientation.z = sin(yaw / 2.0);
  grid.info.origin.orientation.w = cos(yaw / 2.0);
  grid.data.assign(width * height, 0);
  return grid;
}

TEST(NoWaitingZoneTest, test_is_in_zone)
{
  auto grid = makeGrid(10u, 10u, 0.5, 1.0, 1.0);
  grid.data[3 * 10 + 2] = 100;  // cell (2, 3)
  grid.data[5 * 10 + 5] = -1;   // unknown cell (5, 5)

  // center of the marked cell
  EXPECT_TRUE(NoWaitingZone::isInZone(grid, 2.25, 2.75, 1));
  // marked cell with a threshold above its value
  EXPECT_FALSE(NoWaitingZone::isInZone(grid, 2.25, 2.75, 101));
  // free cell
  EXPECT_FALSE(NoWaitingZone::isInZone(grid, 1.25, 1.25, 1));
  // unknown cells are outside of the zone
  EXPECT_FALSE(NoWaitingZone::isInZone(grid, 3.75, 3.75, 1));
  // positions outside of the grid bounds are outside of the zone
  EXPECT_FALSE(NoWaitingZone::isInZone(grid, 0.5, 0.5, 1));
  EXPECT_FALSE(NoWaitingZone::isInZone(grid, 100.0, 100.0, 1));
  EXPECT_FALSE(NoWaitingZone::isInZone(grid, -100.0, -100.0, 1));
}

TEST(NoWaitingZoneTest, test_is_in_zone_rotated)
{
  // grid rotated by 90 degrees: its x axis points along world +y
  auto grid = makeGrid(10u, 10u, 0.5, 1.0, 1.0, M_PI / 2.0);
  grid.data[3 * 10 + 2] = 100;  // cell (2, 3)

  // center of cell (2, 3) in grid frame is (1.25, 1.75), which is
  // world (1.0 - 1.75, 1.0 + 1.25) after rotation
  EXPECT_TRUE(NoWaitingZone::isInZone(grid, -0.75, 2.25, 1));
  // the unrotated position of the marked cell is not in the zone
  EXPECT_FALSE(NoWaitingZone::isInZone(grid, 2.25, 2.75, 1));
}

TEST(NoWaitingZoneTest, test_stop_checker_requires_grid)
{
  nav2_costmap_2d::Costmap2D costmap(100, 100, 0.1, 0.0, 0.0, 0);
  NoWaitingZone zone;
  // no grid received yet: creating a stop checker must fail loudly
  EXPECT_THROW(
    zone.createFreeSpaceStopChecker(&costmap, "map"),
    nav2_core::PlannerException);
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  rclcpp::init(0, nullptr);
  int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
