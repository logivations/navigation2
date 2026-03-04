// Copyright (c) 2023 Dexory
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

#ifndef NAV2_COLLISION_MONITOR__VELOCITY_POLYGON_HPP_
#define NAV2_COLLISION_MONITOR__VELOCITY_POLYGON_HPP_

#include <algorithm>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include "geometry_msgs/msg/polygon_stamped.hpp"
#include "nav2_collision_monitor/polygon.hpp"
#include "nav2_collision_monitor/types.hpp"
#include "nav2_util/lifecycle_node.hpp"
#include "std_msgs/msg/string.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/buffer.h"

namespace nav2_collision_monitor
{

/**
 * @brief Velocity polygon class.
 * This class contains all the points of the polygon and
 * the expected condition of the velocity based polygon.
 */
class VelocityPolygon : public Polygon
{
public:
  /**
   * @brief VelocityPolygon constructor
   * @param node Collision Monitor node pointer
   * @param polygon_name Name of main polygon
   */
  VelocityPolygon(
    const nav2_util::LifecycleNode::WeakPtr & node, const std::string & polygon_name,
    const std::shared_ptr<tf2_ros::Buffer> tf_buffer, const std::string & base_frame_id,
    const tf2::Duration & transform_tolerance);
  /**
   * @brief VelocityPolygon destructor
   */
  virtual ~VelocityPolygon();

  /**
   * @brief Overridden getParameters function for VelocityPolygon parameters
   * @param polygon_sub_topic Not used in VelocityPolygon
   * @param polygon_pub_topic Output name of polygon publishing topic
   * @param footprint_topic Not used in VelocityPolygon
   * @return True if all parameters were obtained or false in failure case
   */
  bool getParameters(
    std::string & /*polygon_sub_topic*/, std::string & polygon_pub_topic,
    std::string & /*footprint_topic*/) override;

  /**
   * @brief Returns current polygon name
   */
  std::string getCurrentSubPolygonName() const { return current_subpolygon_name_; }

  /**
   * @brief Overridden updatePolygon function for VelocityPolygon
   * @param cmd_vel_in Robot twist command input
   */
  void updatePolygon(const Velocity & cmd_vel_in) override;

  /**
   * @brief Validates steering to prevent triggering lidar e-stop zones.
   * Implements Step 2 of the lidar e-stop prevention algorithm.
   * @param cmd_vel_in Desired robot velocity (target)
   * @param odom_vel Current robot velocity from odometry
   * @param collision_points_map Map of source name to collision points
   * @param robot_action Output robot action to modify if steering is restricted
   * @return True if velocity was modified by steering validation
   */
  bool validateSteering(
    const Velocity & cmd_vel_in,
    const Velocity & odom_vel,
    const std::unordered_map<std::string, std::vector<Point>> & collision_points_map,
    Action & robot_action);

protected:
  /**
    * @brief Custom struct to store the parameters of the sub-polygon
    * @param poly_ The points of the sub-polygon
    * @param velocity_polygon_name_ The name of the sub-polygon
    * @param linear_min_ The minimum linear velocity
    * @param linear_max_ The maximum linear velocity
    * @param theta_min_ The minimum angular velocity
    * @param theta_max_ The maximum angular velocity
    * @param steering_angle_min_ The minimum steering angle
    * @param steering_angle_max_ The maxomum steering angle
    * @param direction_end_angle_ The end angle of the direction(For holonomic robot only)
    * @param direction_start_angle_ The start angle of the direction(For holonomic robot only)
    * @param slowdown_ratio_ Robot slowdown (share of its actual speed)
    * @param linear_limit_ Robot linear limit
    * @param angular_limit_ Robot angular limit
    * @param time_before_collision_ Time before collision in seconds
    */
  struct SubPolygonParameter
  {
    std::vector<Point> poly_;
    std::string velocity_polygon_name_;
    double linear_min_;
    double linear_max_;
    double theta_min_;
    double theta_max_;
    double steering_angle_min_;
    double steering_angle_max_;
    bool use_steering_angle_;
    double direction_end_angle_;
    double direction_start_angle_;
    double slowdown_ratio_;
    double linear_limit_;
    double angular_limit_;
    double time_before_collision_;
  };

  /**
   * @brief Check if the velocities and direction is in expected range.
   * @param cmd_vel_in Robot twist command input
   * @param sub_polygon_param Sub polygon parameters
   * @return True if speed and direction is within the condition
   */
  bool isInRange(const Velocity & cmd_vel_in, const SubPolygonParameter & sub_polygon_param);

  /**
   * @brief Compute steering angle from linear and angular velocity
   * @param vel Velocity containing linear.x and angular.z
   * @return Steering angle in radians
   */
  double computeSteeringAngle(const Velocity & vel) const;

  /**
   * @brief Convert baselink speed to steering wheel speed
   * @param linear_vel Baselink linear velocity (twist.linear.x)
   * @param angular_vel Baselink angular velocity (twist.angular.z)
   * @return Steering wheel speed
   */
  double baselinkToSteeringSpeed(double linear_vel, double angular_vel) const;

  /**
   * @brief Convert steering wheel speed to baselink speed
   * @param steering_speed Steering wheel speed
   * @param steering_angle Steering angle in radians
   * @return Baselink speed
   */
  double steeringToBaselinkSpeed(double steering_speed, double steering_angle) const;

  /**
   * @brief Convert steering angle back to angular velocity (twist.angular.z)
   * @param baselink_speed Baselink linear speed
   * @param steering_angle Steering angle in radians
   * @return Angular velocity
   */
  double steeringAngleToTw(double baselink_speed, double steering_angle) const;

  /**
   * @brief Find the field (sub-polygon) matching a given steering wheel speed and steering angle
   * @param steering_wheel_speed Speed in steering wheel frame
   * @param steering_angle Steering angle in radians
   * @return Pointer to matching sub-polygon, or nullptr if none found
   */
  const SubPolygonParameter * findField(
    double steering_wheel_speed, double steering_angle) const;

  /**
   * @brief Find all fields (sub-polygons) matching a given steering angle and direction
   * @param steering_angle Steering angle in radians
   * @param forward If true, return only forward fields (linear_max >= 0);
   *                if false, return only backward fields (linear_min <= 0)
   * @return Vector of pointers to matching sub-polygons, sorted slowest (closest to zero) first
   */
  std::vector<const SubPolygonParameter *> findFieldsForAngle(
    double steering_angle, bool forward) const;

  /**
   * @brief Check if a point is inside a given polygon (arbitrary vertices)
   * @param point Point to check
   * @param vertices Polygon vertices
   * @return True if point is inside polygon
   */
  static bool isPointInsidePoly(const Point & point, const std::vector<Point> & vertices);

  /**
   * @brief Get number of collision points inside a specific sub-polygon
   * @param sub_polygon Sub-polygon to check against
   * @param collision_points_map Map of source name to collision points
   * @return Number of collision points inside the sub-polygon
   */
  int getPointsInsideSubPolygon(
    const SubPolygonParameter & sub_polygon,
    const std::unordered_map<std::string, std::vector<Point>> & collision_points_map) const;

  // Clock
  rclcpp::Clock::SharedPtr clock_;
  // Current subpolygon name
  std::string current_subpolygon_name_;
  // Variables
  /// @brief Flag to indicate if the robot is holonomic
  bool holonomic_;
  /// @brief Current steering angle
  double current_steering_angle_;
  /// @brief Distance between front and rear axes
  double wheelbase_;
  /// @brief Speed below which steering is freely allowed
  double low_speed_threshold_;
  /// @brief Vector to store the parameters of the sub-polygon
  std::vector<SubPolygonParameter> sub_polygons_;
};  // class VelocityPolygon

}  // namespace nav2_collision_monitor

#endif  // NAV2_COLLISION_MONITOR__VELOCITY_POLYGON_HPP_
