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

#include "nav2_collision_monitor/velocity_polygon.hpp"

#include <algorithm>
#include <cmath>

#include "nav2_util/node_utils.hpp"

namespace nav2_collision_monitor
{

VelocityPolygon::VelocityPolygon(
  const nav2_util::LifecycleNode::WeakPtr & node, const std::string & polygon_name,
  const std::shared_ptr<tf2_ros::Buffer> tf_buffer, const std::string & base_frame_id,
  const tf2::Duration & transform_tolerance)
: Polygon::Polygon(node, polygon_name, tf_buffer, base_frame_id, transform_tolerance)
{
  RCLCPP_INFO(logger_, "[%s]: Creating VelocityPolygon", polygon_name_.c_str());
}

VelocityPolygon::~VelocityPolygon()
{
  RCLCPP_INFO(logger_, "[%s]: Destroying VelocityPolygon", polygon_name_.c_str());
}

bool VelocityPolygon::getParameters(
  std::string & polygon_sub_topic,
  std::string & polygon_pub_topic,
  std::string & footprint_topic)
{
  auto node = node_.lock();
  if (!node) {
    throw std::runtime_error{"Failed to lock node"};
  }
  clock_ = node->get_clock();

  if (!getCommonParameters(polygon_sub_topic, polygon_pub_topic, footprint_topic, false)) {
    return false;
  }

  try {
    // Get velocity_polygons parameter
    nav2_util::declare_parameter_if_not_declared(
      node, polygon_name_ + ".velocity_polygons", rclcpp::PARAMETER_STRING_ARRAY);
    std::vector<std::string> velocity_polygons =
      node->get_parameter(polygon_name_ + ".velocity_polygons").as_string_array();

    // holonomic param
    nav2_util::declare_parameter_if_not_declared(
      node, polygon_name_ + ".holonomic", rclcpp::ParameterValue(false));
    holonomic_ = node->get_parameter(polygon_name_ + ".holonomic").as_bool();

    nav2_util::declare_parameter_if_not_declared(
      node, polygon_name_ + ".wheelbase", rclcpp::ParameterValue(1.0));
    wheelbase_ = node->get_parameter(polygon_name_ + ".wheelbase").as_double();

    nav2_util::declare_parameter_if_not_declared(
      node, polygon_name_ + ".low_speed_threshold", rclcpp::ParameterValue(0.1));
    low_speed_threshold_ = node->get_parameter(
      polygon_name_ + ".low_speed_threshold").as_double();

    for (std::string velocity_polygon_name : velocity_polygons) {
      // polygon points parameter
      std::vector<Point> poly;
      nav2_util::declare_parameter_if_not_declared(
        node, polygon_name_ + "." + velocity_polygon_name + ".points", rclcpp::PARAMETER_STRING);
      std::string poly_string =
        node->get_parameter(polygon_name_ + "." + velocity_polygon_name + ".points").as_string();

      if (!getPolygonFromString(poly_string, poly)) {
        return false;
      }

      // linear_min param
      double linear_min;
      nav2_util::declare_parameter_if_not_declared(
        node, polygon_name_ + "." + velocity_polygon_name + ".linear_min",
        rclcpp::PARAMETER_DOUBLE);
      linear_min = node->get_parameter(polygon_name_ + "." + velocity_polygon_name + ".linear_min")
        .as_double();

      // linear_max param
      double linear_max;
      nav2_util::declare_parameter_if_not_declared(
        node, polygon_name_ + "." + velocity_polygon_name + ".linear_max",
        rclcpp::PARAMETER_DOUBLE);
      linear_max = node->get_parameter(polygon_name_ + "." + velocity_polygon_name + ".linear_max")
        .as_double();

      const std::string steering_min_param = polygon_name_ + "." + velocity_polygon_name + ".steering_angle_min";
      const std::string steering_max_param = polygon_name_ + "." + velocity_polygon_name + ".steering_angle_max";
      const std::string theta_min_param = polygon_name_ + "." + velocity_polygon_name + ".theta_min";
      const std::string theta_max_param = polygon_name_ + "." + velocity_polygon_name + ".theta_max";

      bool use_steering_angle = false;
      double steering_angle_min = 0.0;
      double steering_angle_max = 0.0;
      double theta_min = 0.0;
      double theta_max = 0.0;

      bool has_steering_params = false;
      bool has_theta_params = false;

      try {
        nav2_util::declare_parameter_if_not_declared(
          node, steering_min_param, rclcpp::PARAMETER_DOUBLE);
        nav2_util::declare_parameter_if_not_declared(
          node, steering_max_param, rclcpp::PARAMETER_DOUBLE);

        steering_angle_min = node->get_parameter(steering_min_param).as_double();
        steering_angle_max = node->get_parameter(steering_max_param).as_double();
        has_steering_params = true;
      } catch (const rclcpp::exceptions::ParameterNotDeclaredException &) {
        RCLCPP_DEBUG(logger_, "steering_angle parameters not found, will check theta parameters");
      } catch (const rclcpp::exceptions::ParameterUninitializedException &) {
        RCLCPP_DEBUG(logger_, "steering_angle parameters not initialized");
      }

      if (!has_steering_params) {
        try {
          nav2_util::declare_parameter_if_not_declared(
            node, theta_min_param, rclcpp::PARAMETER_DOUBLE);
          nav2_util::declare_parameter_if_not_declared(
            node, theta_max_param, rclcpp::PARAMETER_DOUBLE);

          theta_min = node->get_parameter(theta_min_param).as_double();
          theta_max = node->get_parameter(theta_max_param).as_double();
          has_theta_params = true;
        } catch (const rclcpp::exceptions::ParameterNotDeclaredException &) {
          RCLCPP_DEBUG(logger_, "Theta parameters not found");
        } catch (const rclcpp::exceptions::ParameterUninitializedException &) {
          RCLCPP_DEBUG(logger_, "Theta parameters not initialized");
        }
      }

      if (has_steering_params) {
        use_steering_angle = true;
        RCLCPP_INFO(
          logger_,
          "[%s]: Using steering_angle parameters for %s (min: %f, max: %f)",
          polygon_name_.c_str(),
          velocity_polygon_name.c_str(),
          steering_angle_min,
          steering_angle_max
        );
      } else if (has_theta_params) {
        use_steering_angle = false;
        RCLCPP_INFO(
          logger_,
          "[%s]: Using theta parameters for %s (min: %f, max: %f)",
          polygon_name_.c_str(),
          velocity_polygon_name.c_str(),
          theta_min,
          theta_max
        );
      } else {
        RCLCPP_ERROR(
          logger_,
          "[%s]: Neither steering_angle parameters nor theta parameters are set for %s",
          polygon_name_.c_str(),
          velocity_polygon_name.c_str()
        );

        return false;
      }

      // direction_end_angle param and direction_start_angle param
      double direction_end_angle = 0.0;
      double direction_start_angle = 0.0;
      if (holonomic_) {
        nav2_util::declare_parameter_if_not_declared(
          node, polygon_name_ + "." + velocity_polygon_name + ".direction_end_angle",
          rclcpp::ParameterValue(M_PI));
        direction_end_angle =
          node->get_parameter(polygon_name_ + "." + velocity_polygon_name + ".direction_end_angle")
          .as_double();

        nav2_util::declare_parameter_if_not_declared(
          node, polygon_name_ + "." + velocity_polygon_name + ".direction_start_angle",
          rclcpp::ParameterValue(-M_PI));
        direction_start_angle =
          node
          ->get_parameter(polygon_name_ + "." + velocity_polygon_name + ".direction_start_angle")
          .as_double();
      }

      double slowdown_ratio = 0.0;
      if (action_type_ == SLOWDOWN) {
        nav2_util::declare_parameter_if_not_declared(
          node, polygon_name_ + "." + velocity_polygon_name + ".slowdown_ratio", rclcpp::ParameterValue(
            0.5));
        slowdown_ratio = node->get_parameter(
          polygon_name_ + "." + velocity_polygon_name + ".slowdown_ratio").as_double();
      }

      double linear_limit = 0.0;
      double angular_limit = 0.0;
      if (action_type_ == LIMIT) {
        nav2_util::declare_parameter_if_not_declared(
          node, polygon_name_ + "." + velocity_polygon_name + ".linear_limit", rclcpp::ParameterValue(
            0.5));
        linear_limit = node->get_parameter(
          polygon_name_ + "." + velocity_polygon_name + ".linear_limit").as_double();
        nav2_util::declare_parameter_if_not_declared(
          node, polygon_name_ + "." + velocity_polygon_name + ".angular_limit", rclcpp::ParameterValue(
            0.5));
        angular_limit = node->get_parameter(
          polygon_name_ + "." + velocity_polygon_name + ".angular_limit").as_double();
      }

      double time_before_collision = 0.0;
      if (action_type_ == APPROACH) {
        nav2_util::declare_parameter_if_not_declared(
          node, polygon_name_ + "." + velocity_polygon_name + ".time_before_collision", rclcpp::ParameterValue(
            2.0));
        time_before_collision = node->get_parameter(
          polygon_name_ + "." + velocity_polygon_name + ".time_before_collision").as_double();
        nav2_util::declare_parameter_if_not_declared(
          node, polygon_name_ + "." + velocity_polygon_name + ".simulation_time_step", rclcpp::ParameterValue(
            0.1));
      }

      SubPolygonParameter sub_polygon = {
        poly, velocity_polygon_name, linear_min, linear_max, theta_min,
        theta_max, steering_angle_min, steering_angle_max, use_steering_angle,
        direction_end_angle, direction_start_angle,
        slowdown_ratio, linear_limit, angular_limit, time_before_collision,
      };

      sub_polygons_.push_back(sub_polygon);
    }
  } catch (const std::exception & ex) {
    RCLCPP_ERROR(
      logger_, "[%s]: Error while getting polygon parameters: %s", polygon_name_.c_str(),
      ex.what());
    return false;
  }
  return true;
}

void VelocityPolygon::updatePolygon(const Velocity & cmd_vel_in)
{
  for (auto & sub_polygon : sub_polygons_) {
    if (isInRange(cmd_vel_in, sub_polygon)) {
      // Set the polygon that is within the speed range
      poly_ = sub_polygon.poly_;

      current_subpolygon_name_ = sub_polygon.velocity_polygon_name_;

      // Update visualization polygon
      polygon_.polygon.points.clear();
      for (const Point & p : poly_) {
        geometry_msgs::msg::Point32 p_s;
        p_s.x = p.x;
        p_s.y = p.y;
        // p_s.z will remain 0.0
        polygon_.polygon.points.push_back(p_s);
      }

      slowdown_ratio_ = sub_polygon.slowdown_ratio_;
      if (sub_polygon.use_steering_angle_) {
        // Convert linear_limit from steering wheel speed to baselink speed
        linear_limit_ = steeringToBaselinkSpeed(
          sub_polygon.linear_limit_, current_steering_angle_);
      } else {
        linear_limit_ = sub_polygon.linear_limit_;
      }
      angular_limit_ = sub_polygon.angular_limit_;
      time_before_collision_ = sub_polygon.time_before_collision_;

      return;
    }
  }

  current_subpolygon_name_ = "none";

  // Log for uncovered velocity
  RCLCPP_WARN_THROTTLE(
    logger_, *clock_, 2.0,
    "Velocity is not covered by any of the velocity polygons. x: %.3f y: %.3f tw: %.3f ",
    cmd_vel_in.x, cmd_vel_in.y, cmd_vel_in.tw);
  return;
}

bool VelocityPolygon::isInRange(const Velocity & cmd_vel_in, const SubPolygonParameter & sub_polygon) {
  if (sub_polygon.use_steering_angle_) {
    // Compute steering angle first (needed for speed conversion)
    if (std::abs(cmd_vel_in.x) < 1e-6) {
      current_steering_angle_ = (std::abs(cmd_vel_in.tw) < 1e-6) ?
                               0.0 :
                               (cmd_vel_in.tw > 0 ? M_PI/2 : -M_PI/2);
    } else {
      double angular_vel = cmd_vel_in.tw;
      if (cmd_vel_in.x < 0) {
        angular_vel = -angular_vel;
      }
      current_steering_angle_ = std::atan2(wheelbase_ * angular_vel, std::abs(cmd_vel_in.x));
    }

    // Convert baselink speed to steering wheel speed: v_steering = v_baselink / cos(steering_angle)
    double steering_wheel_speed = baselinkToSteeringSpeed(cmd_vel_in.x, current_steering_angle_);

    RCLCPP_DEBUG(
      logger_,
      "Calculated steering angle: %.2f (limits: %.2f to %.2f), "
      "baselink_vel: %.2f, steering_wheel_speed: %.2f, angular_vel: %.2f",
      current_steering_angle_,
      sub_polygon.steering_angle_min_,
      sub_polygon.steering_angle_max_,
      cmd_vel_in.x,
      steering_wheel_speed,
      cmd_vel_in.tw
    );

    // Check linear range using steering wheel speed
    bool in_range = steering_wheel_speed <= sub_polygon.linear_max_ &&
                    steering_wheel_speed >= sub_polygon.linear_min_;

    if (!in_range) {
      return false;
    }

    // Check steering angle range
    in_range &= current_steering_angle_ <= sub_polygon.steering_angle_max_ &&
                current_steering_angle_ >= sub_polygon.steering_angle_min_;

    return in_range;
  }

  // Non-steering-angle mode: use baselink speed directly
  bool in_range = cmd_vel_in.x <= sub_polygon.linear_max_ &&
                  cmd_vel_in.x >= sub_polygon.linear_min_;

  if (!in_range) {
    return false;
  }

  in_range &= cmd_vel_in.tw <= sub_polygon.theta_max_ &&
              cmd_vel_in.tw >= sub_polygon.theta_min_;

  if (holonomic_) {
    // Additionally check if moving direction in angle range(start -> end) for holonomic case
    const double direction = std::atan2(cmd_vel_in.y, cmd_vel_in.x);
    if (sub_polygon.direction_start_angle_ <= sub_polygon.direction_end_angle_) {
      in_range &=
        (direction >= sub_polygon.direction_start_angle_ &&
        direction <= sub_polygon.direction_end_angle_);
    } else {
      in_range &=
        (direction >= sub_polygon.direction_start_angle_ ||
        direction <= sub_polygon.direction_end_angle_);
    }
  }

  return in_range;
}

double VelocityPolygon::baselinkToSteeringSpeed(
  double baselink_speed, double steering_angle) const
{
  double cos_sa = std::cos(steering_angle);
  if (std::abs(cos_sa) < 1e-6) {
    // Near 90 degree steering — return a very large speed to signal out-of-range
    return (baselink_speed >= 0.0 ? 1e6 : -1e6);
  }
  return baselink_speed / cos_sa;
}

double VelocityPolygon::steeringToBaselinkSpeed(
  double steering_speed, double steering_angle) const
{
  return steering_speed * std::cos(steering_angle);
}

double VelocityPolygon::steeringAngleToTw(
  double baselink_speed, double steering_angle) const
{
  // tw = tan(angle) * |v| / wheelbase, sign-corrected for reverse
  double tw = std::tan(steering_angle) * std::abs(baselink_speed) / wheelbase_;
  if (baselink_speed < 0.0) {
    tw = -tw;
  }
  return tw;
}

const VelocityPolygon::SubPolygonParameter * VelocityPolygon::findBucket(
  double steering_wheel_speed, double steering_angle) const
{
  for (const auto & sp : sub_polygons_) {
    if (!sp.use_steering_angle_) {
      continue;
    }
    if (steering_wheel_speed >= sp.linear_min_ && steering_wheel_speed <= sp.linear_max_ &&
      steering_angle >= sp.steering_angle_min_ && steering_angle <= sp.steering_angle_max_)
    {
      return &sp;
    }
  }
  return nullptr;
}

std::vector<const VelocityPolygon::SubPolygonParameter *>
VelocityPolygon::findBucketsForAngle(double steering_angle) const
{
  std::vector<const SubPolygonParameter *> result;
  for (const auto & sp : sub_polygons_) {
    if (!sp.use_steering_angle_) {
      continue;
    }
    if (steering_angle >= sp.steering_angle_min_ && steering_angle <= sp.steering_angle_max_) {
      result.push_back(&sp);
    }
  }
  // Sort ascending by linear_min_ (slowest first)
  std::sort(result.begin(), result.end(),
    [](const SubPolygonParameter * a, const SubPolygonParameter * b) {
      return a->linear_min_ < b->linear_min_;
    });
  return result;
}

bool VelocityPolygon::isPointInsidePoly(
  const Point & point, const std::vector<Point> & vertices)
{
  // Ray-casting algorithm (same as Polygon::isPointInside but for arbitrary vertices)
  const int poly_size = static_cast<int>(vertices.size());
  int i, j;
  bool res = false;

  i = poly_size - 1;
  for (j = 0; j < poly_size; j++) {
    if ((point.y <= vertices[i].y) == (point.y > vertices[j].y)) {
      const double x_inter = vertices[i].x +
        (point.y - vertices[i].y) * (vertices[j].x - vertices[i].x) /
        (vertices[j].y - vertices[i].y);
      if (x_inter > point.x) {
        res = !res;
      }
    }
    i = j;
  }
  return res;
}

int VelocityPolygon::getPointsInsideSubPolygon(
  const SubPolygonParameter & sub_polygon,
  const std::unordered_map<std::string, std::vector<Point>> & collision_points_map) const
{
  int num = 0;
  std::vector<std::string> polygon_sources_names = getSourcesNames();

  for (const auto & source_name : polygon_sources_names) {
    const auto & iter = collision_points_map.find(source_name);
    if (iter != collision_points_map.end()) {
      for (const auto & point : iter->second) {
        if (isPointInsidePoly(point, sub_polygon.poly_)) {
          num++;
        }
      }
    }
  }

  return num;
}

bool VelocityPolygon::validateSteering(
  const Velocity & cmd_vel_in,
  const Velocity & odom_vel,
  const std::unordered_map<std::string, std::vector<Point>> & collision_points_map,
  Action & robot_action)
{
  // Only applies to steering-angle-based velocity polygons
  if (sub_polygons_.empty() || !sub_polygons_[0].use_steering_angle_) {
    return false;
  }

  const double target_speed = cmd_vel_in.x;
  const double current_speed = odom_vel.x;

  // Compute target steering angle
  double target_steering_angle;
  if (std::abs(cmd_vel_in.x) < 1e-6) {
    target_steering_angle = (std::abs(cmd_vel_in.tw) < 1e-6) ?
      0.0 : (cmd_vel_in.tw > 0 ? M_PI / 2 : -M_PI / 2);
  } else {
    double angular_vel = cmd_vel_in.tw;
    if (cmd_vel_in.x < 0) {
      angular_vel = -angular_vel;
    }
    target_steering_angle = std::atan2(wheelbase_ * angular_vel, std::abs(cmd_vel_in.x));
  }

  // Compute current steering angle from odometry
  double current_sa;
  if (std::abs(odom_vel.x) < 1e-6) {
    current_sa = (std::abs(odom_vel.tw) < 1e-6) ?
      0.0 : (odom_vel.tw > 0 ? M_PI / 2 : -M_PI / 2);
  } else {
    double angular_vel = odom_vel.tw;
    if (odom_vel.x < 0) {
      angular_vel = -angular_vel;
    }
    current_sa = std::atan2(wheelbase_ * angular_vel, std::abs(odom_vel.x));
  }

  bool modified = false;
  Velocity result_vel = robot_action.req_vel;

  // Check if speed crosses zero (direction reversal)
  bool crosses_zero = (target_speed > 0 && current_speed < 0) ||
    (target_speed < 0 && current_speed > 0);

  if (crosses_zero) {
    if (std::abs(current_speed) > low_speed_threshold_) {
      // Must decelerate first — clamp tw to maintain current steering angle
      result_vel.tw = steeringAngleToTw(result_vel.x, current_sa);
      modified = true;
    }
    // else: abs(current) < threshold → allow steering freely
    if (modified) {
      robot_action.req_vel = result_vel;
      robot_action.polygon_name = polygon_name_;
      robot_action.action_type = LIMIT;
    }
    return modified;
  }

  // Speed does NOT cross zero
  // 1. Both abs(target) and abs(current) below threshold → done
  if (std::abs(target_speed) < low_speed_threshold_ &&
    std::abs(current_speed) < low_speed_threshold_)
  {
    return false;
  }

  double target_sw_speed = baselinkToSteeringSpeed(target_speed, target_steering_angle);
  double current_sw_speed = baselinkToSteeringSpeed(current_speed, current_sa);

  const SubPolygonParameter * current_bucket = findBucket(current_sw_speed, current_sa);
  const SubPolygonParameter * target_bucket = findBucket(target_sw_speed, target_steering_angle);

  // 2. Target angle in same bucket as current angle
  if (target_bucket != nullptr && current_bucket != nullptr &&
    target_bucket == current_bucket)
  {
    // Same bucket
    if (std::abs(target_sw_speed) > std::abs(current_sw_speed)) {
      // Accelerating — check next faster bucket at target angle for collision
      auto buckets_at_angle = findBucketsForAngle(target_steering_angle);
      for (size_t i = 0; i < buckets_at_angle.size(); i++) {
        if (buckets_at_angle[i] == current_bucket && i + 1 < buckets_at_angle.size()) {
          const SubPolygonParameter * next_bucket = buckets_at_angle[i + 1];
          if (getPointsInsideSubPolygon(*next_bucket, collision_points_map) >= min_points_) {
            // Collision in next bucket — limit speed to current bucket's linear_max
            double max_baselink = steeringToBaselinkSpeed(
              current_bucket->linear_max_, current_sa);
            if (std::abs(result_vel.x) > std::abs(max_baselink)) {
              result_vel.x = (result_vel.x >= 0) ? std::abs(max_baselink) : -std::abs(max_baselink);
              result_vel.tw = steeringAngleToTw(result_vel.x, target_steering_angle);
              modified = true;
            }
          }
          break;
        }
      }
    }
    // Otherwise (decelerating or same speed) → done
    if (modified) {
      robot_action.req_vel = result_vel;
      robot_action.polygon_name = polygon_name_;
      robot_action.action_type = LIMIT;
    }
    return modified;
  }

  // 3. Target angle in different bucket
  // Find all buckets at target angle
  auto target_buckets = findBucketsForAngle(target_steering_angle);
  if (target_buckets.empty()) {
    return false;
  }

  // Find the fastest bucket that covers max(|current|, |target|) speed
  double max_sw_speed = std::max(std::abs(current_sw_speed), std::abs(target_sw_speed));

  // Find the valid bucket: start from fastest covering bucket, walk down
  const SubPolygonParameter * valid_bucket = nullptr;
  int start_idx = static_cast<int>(target_buckets.size()) - 1;

  // Find the starting bucket (fastest that covers max_sw_speed)
  for (int i = start_idx; i >= 0; i--) {
    if (max_sw_speed >= target_buckets[i]->linear_min_ &&
      max_sw_speed <= target_buckets[i]->linear_max_)
    {
      start_idx = i;
      break;
    }
    // If max_sw_speed is beyond all buckets, start from the fastest
    if (i == 0) {
      start_idx = static_cast<int>(target_buckets.size()) - 1;
    }
  }

  // Check if fastest bucket is collision-free → done
  if (getPointsInsideSubPolygon(*target_buckets[start_idx], collision_points_map) < min_points_) {
    return false;
  }

  // Walk down speed buckets until a collision-free one is found
  for (int i = start_idx; i >= 0; i--) {
    if (getPointsInsideSubPolygon(*target_buckets[i], collision_points_map) < min_points_) {
      valid_bucket = target_buckets[i];
      break;
    }
  }

  if (valid_bucket == nullptr) {
    // No valid bucket exists → STOP
    robot_action.req_vel.x = 0.0;
    robot_action.req_vel.y = 0.0;
    robot_action.req_vel.tw = 0.0;
    robot_action.polygon_name = polygon_name_;
    robot_action.action_type = STOP;
    return true;
  }

  // 4. Decelerate to valid bucket
  // Limit steering angle to boundary of current bucket (toward target direction)
  if (current_bucket != nullptr) {
    double limited_sa;
    if (target_steering_angle > current_sa) {
      limited_sa = current_bucket->steering_angle_max_;
    } else {
      limited_sa = current_bucket->steering_angle_min_;
    }
    result_vel.tw = steeringAngleToTw(result_vel.x, limited_sa);
  }

  // Limit speed to valid bucket's linear_max (converted to baselink)
  double valid_max_baselink = steeringToBaselinkSpeed(
    valid_bucket->linear_max_, target_steering_angle);
  if (std::abs(result_vel.x) > std::abs(valid_max_baselink)) {
    result_vel.x = (result_vel.x >= 0) ? std::abs(valid_max_baselink) :
      -std::abs(valid_max_baselink);
    // Recalculate tw for the new speed
    if (current_bucket != nullptr) {
      double limited_sa;
      if (target_steering_angle > current_sa) {
        limited_sa = current_bucket->steering_angle_max_;
      } else {
        limited_sa = current_bucket->steering_angle_min_;
      }
      result_vel.tw = steeringAngleToTw(result_vel.x, limited_sa);
    }
  }

  robot_action.req_vel = result_vel;
  robot_action.polygon_name = polygon_name_;
  robot_action.action_type = LIMIT;
  return true;
}

}  // namespace nav2_collision_monitor
