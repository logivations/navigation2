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
#include <limits>

#include "nav2_ros_common/node_utils.hpp"

namespace nav2_collision_monitor
{

VelocityPolygon::VelocityPolygon(
  const nav2::LifecycleNode::WeakPtr & node, const std::string & polygon_name,
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
    std::vector<std::string> velocity_polygons =
      node->declare_or_get_parameter<std::vector<std::string>>(
      polygon_name_ + ".velocity_polygons");

    // holonomic param
    holonomic_ = node->declare_or_get_parameter(
      polygon_name_ + ".holonomic", false);

    wheelbase_ = node->declare_or_get_parameter(polygon_name_ + ".wheelbase", 1.0);

    low_speed_threshold_ = node->declare_or_get_parameter(
      polygon_name_ + ".low_speed_threshold", 0.1);

    for (std::string velocity_polygon_name : velocity_polygons) {
      // polygon points parameter
      std::vector<Point> poly;
      std::string poly_string =
        node->declare_or_get_parameter<std::string>(
        polygon_name_ + "." + velocity_polygon_name + ".points");

      if (!getPolygonFromString(poly_string, poly)) {
        return false;
      }

      // linear_min param
      double linear_min = node->declare_or_get_parameter<double>(
        polygon_name_ + "." + velocity_polygon_name + ".linear_min");

      // linear_max param
      double linear_max = node->declare_or_get_parameter<double>(
        polygon_name_ + "." + velocity_polygon_name + ".linear_max");

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
        steering_angle_min = node->declare_or_get_parameter<double>(steering_min_param);
        steering_angle_max = node->declare_or_get_parameter<double>(steering_max_param);
        has_steering_params = true;
      } catch (const rclcpp::exceptions::ParameterNotDeclaredException &) {
        RCLCPP_DEBUG(logger_, "steering_angle parameters not found, will check theta parameters");
      } catch (const rclcpp::exceptions::ParameterUninitializedException &) {
        RCLCPP_DEBUG(logger_, "steering_angle parameters not initialized");
      } catch (const rclcpp::exceptions::InvalidParameterValueException &) {
        RCLCPP_DEBUG(logger_, "steering_angle parameters not set");
      }

      if (!has_steering_params) {
        try {
          theta_min = node->declare_or_get_parameter<double>(theta_min_param);
          theta_max = node->declare_or_get_parameter<double>(theta_max_param);
          has_theta_params = true;
        } catch (const rclcpp::exceptions::ParameterNotDeclaredException &) {
          RCLCPP_DEBUG(logger_, "Theta parameters not found");
        } catch (const rclcpp::exceptions::ParameterUninitializedException &) {
          RCLCPP_DEBUG(logger_, "Theta parameters not initialized");
        } catch (const rclcpp::exceptions::InvalidParameterValueException &) {
          RCLCPP_DEBUG(logger_, "Theta parameters not set");
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
        direction_end_angle = node->declare_or_get_parameter(
          polygon_name_ + "." + velocity_polygon_name + ".direction_end_angle", M_PI);

        direction_start_angle = node->declare_or_get_parameter(
          polygon_name_ + "." + velocity_polygon_name + ".direction_start_angle", -M_PI);
      }

      double slowdown_ratio = 0.0;
      if (action_type_ == SLOWDOWN) {
        slowdown_ratio = node->declare_or_get_parameter(
          polygon_name_ + "." + velocity_polygon_name + ".slowdown_ratio", 0.5);
      }

      double linear_limit = 0.0;
      double angular_limit = 0.0;
      if (action_type_ == LIMIT) {
        linear_limit = node->declare_or_get_parameter(
          polygon_name_ + "." + velocity_polygon_name + ".linear_limit", 0.5);
        angular_limit = node->declare_or_get_parameter(
          polygon_name_ + "." + velocity_polygon_name + ".angular_limit", 0.5);
      }

      double time_before_collision = 0.0;
      if (action_type_ == APPROACH) {
        time_before_collision = node->declare_or_get_parameter(
          polygon_name_ + "." + velocity_polygon_name + ".time_before_collision", 2.0);
        node->declare_or_get_parameter(
          polygon_name_ + "." + velocity_polygon_name + ".simulation_time_step", 0.1);
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

  steering_debug_pub_ = node->create_publisher<nav2_msgs::msg::SteeringValidationDebug>(
    "~/steering_validation_debug", rclcpp::QoS(1));

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

bool VelocityPolygon::isInRange(
  const Velocity & cmd_vel_in, const SubPolygonParameter & sub_polygon)
{
  if (sub_polygon.use_steering_angle_) {
    current_steering_angle_ = computeSteeringAngle(cmd_vel_in);

    // Convert baselink speed to steering wheel speed
    double steering_wheel_speed = baselinkToSteeringSpeed(cmd_vel_in.x, cmd_vel_in.tw);

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
    // 2. For holonomic robots: use speed magnitude + direction
    const double magnitude = std::hypot(cmd_vel_in.x, cmd_vel_in.y);
    // Direction is undefined at rest; choose 0 and rely on configured direction ranges.
    const double direction = (magnitude > 0.0) ? std::atan2(cmd_vel_in.y, cmd_vel_in.x) : 0.0;

    // Linear range on speed magnitude
    in_range &= (magnitude <= sub_polygon.linear_max_ &&
      magnitude >= sub_polygon.linear_min_);

    // Direction range
    if (sub_polygon.direction_start_angle_ <= sub_polygon.direction_end_angle_) {
      in_range &=
        (direction >= sub_polygon.direction_start_angle_ &&
        direction <= sub_polygon.direction_end_angle_);
    } else {
      in_range &=
        (direction >= sub_polygon.direction_start_angle_ ||
        direction <= sub_polygon.direction_end_angle_);
    }
  } else {
    // 3. Non-holonomic: keep x-based behavior
    in_range &=
      (cmd_vel_in.x <= sub_polygon.linear_max_ &&
      cmd_vel_in.x >= sub_polygon.linear_min_);
  }

  return in_range;
}

double VelocityPolygon::computeSteeringAngle(const Velocity & vel) const
{
  if (std::abs(vel.x) < 1e-6) {
    return (std::abs(vel.tw) < 1e-6) ? 0.0 : (vel.tw > 0 ? M_PI / 2 : -M_PI / 2);
  }
  double angular_vel = vel.tw;
  if (vel.x < 0) {
    angular_vel = -angular_vel;
  }
  return std::atan2(wheelbase_ * angular_vel, std::abs(vel.x));
}

double VelocityPolygon::baselinkToSteeringSpeed(
  double linear_vel, double angular_vel) const
{
  double magnitude = std::hypot(linear_vel, wheelbase_ * angular_vel);
  if (linear_vel < 0.0) {
    return -magnitude;
  }
  return magnitude;
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

const VelocityPolygon::SubPolygonParameter * VelocityPolygon::findField(
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
VelocityPolygon::findFieldsForAngle(double steering_angle, bool forward) const
{
  std::vector<const SubPolygonParameter *> result;
  for (const auto & sp : sub_polygons_) {
    if (!sp.use_steering_angle_) {
      continue;
    }
    if (steering_angle >= sp.steering_angle_min_ && steering_angle <= sp.steering_angle_max_) {
      if (forward ? (sp.linear_max_ > 0) : (sp.linear_min_ < 0)) {
        result.push_back(&sp);
      }
    }
  }
  // Sort by speed magnitude ascending (slowest/closest to zero first)
  std::sort(result.begin(), result.end(),
    [](const SubPolygonParameter * a, const SubPolygonParameter * b) {
      return std::abs(a->linear_min_) < std::abs(b->linear_min_);
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

  nav2_msgs::msg::SteeringValidationDebug debug_msg;
  debug_msg.header.stamp = clock_->now();
  debug_msg.polygon_name = polygon_name_;
  debug_msg.steering_angle_limit = std::numeric_limits<double>::quiet_NaN();
  debug_msg.speed_limit_applied = 0.0;
  debug_msg.next_field_collision_pts = -1;
  debug_msg.neighbour_collision_pts = -1;

  // Step 1 diagnostics: re-check the polygon state from processStopSlowdownLimit
  debug_msg.step1_active_sub_polygon = current_subpolygon_name_;
  debug_msg.step1_shape_set = isShapeSet();
  debug_msg.step1_min_points = min_points_;
  debug_msg.step1_points_inside = getPointsInside(collision_points_map);
  debug_msg.step1_linear_limit = linear_limit_;
  debug_msg.step1_angular_limit = angular_limit_;
  debug_msg.step1_action_type = robot_action.action_type;
  debug_msg.step1_req_vel_x = robot_action.req_vel.x;
  debug_msg.step1_req_vel_y = robot_action.req_vel.y;
  debug_msg.step1_req_vel_tw = robot_action.req_vel.tw;

  // Source filtering diagnostics
  debug_msg.step1_configured_sources = getSourcesNames();
  debug_msg.step1_polygon_vertex_count = static_cast<int32_t>(poly_.size());
  int configured_pts = 0;
  for (const auto & src_name : debug_msg.step1_configured_sources) {
    auto it = collision_points_map.find(src_name);
    if (it != collision_points_map.end()) {
      configured_pts += static_cast<int>(it->second.size());
    }
  }
  debug_msg.step1_configured_source_pts = configured_pts;
  int all_pts = 0;
  for (const auto & kv : collision_points_map) {
    all_pts += static_cast<int>(kv.second.size());
  }
  debug_msg.step1_all_source_pts = all_pts;

  // Bounding box of polygon vertices
  double poly_min_x = std::numeric_limits<double>::max();
  double poly_max_x = std::numeric_limits<double>::lowest();
  double poly_min_y = std::numeric_limits<double>::max();
  double poly_max_y = std::numeric_limits<double>::lowest();
  for (const auto & v : poly_) {
    poly_min_x = std::min(poly_min_x, v.x);
    poly_max_x = std::max(poly_max_x, v.x);
    poly_min_y = std::min(poly_min_y, v.y);
    poly_max_y = std::max(poly_max_y, v.y);
  }
  debug_msg.step1_poly_min_x = poly_min_x;
  debug_msg.step1_poly_max_x = poly_max_x;
  debug_msg.step1_poly_min_y = poly_min_y;
  debug_msg.step1_poly_max_y = poly_max_y;

  // Bounding box of collision points from configured sources
  double pts_min_x = std::numeric_limits<double>::max();
  double pts_max_x = std::numeric_limits<double>::lowest();
  double pts_min_y = std::numeric_limits<double>::max();
  double pts_max_y = std::numeric_limits<double>::lowest();
  for (const auto & src_name : debug_msg.step1_configured_sources) {
    auto it = collision_points_map.find(src_name);
    if (it != collision_points_map.end()) {
      for (const auto & pt : it->second) {
        pts_min_x = std::min(pts_min_x, pt.x);
        pts_max_x = std::max(pts_max_x, pt.x);
        pts_min_y = std::min(pts_min_y, pt.y);
        pts_max_y = std::max(pts_max_y, pt.y);
      }
    }
  }
  debug_msg.step1_pts_min_x = pts_min_x;
  debug_msg.step1_pts_max_x = pts_max_x;
  debug_msg.step1_pts_min_y = pts_min_y;
  debug_msg.step1_pts_max_y = pts_max_y;

  // Count points within the polygon's bounding box
  int pts_in_bbox = 0;
  for (const auto & src_name : debug_msg.step1_configured_sources) {
    auto it = collision_points_map.find(src_name);
    if (it != collision_points_map.end()) {
      for (const auto & pt : it->second) {
        if (pt.x >= poly_min_x && pt.x <= poly_max_x &&
          pt.y >= poly_min_y && pt.y <= poly_max_y)
        {
          pts_in_bbox++;
        }
      }
    }
  }
  debug_msg.step1_pts_in_bbox = pts_in_bbox;

  const double target_speed = cmd_vel_in.x;
  const double current_speed = odom_vel.x;

  const double target_steering_angle = computeSteeringAngle(cmd_vel_in);
  const double current_sa = computeSteeringAngle(odom_vel);

  debug_msg.target_speed = target_speed;
  debug_msg.current_speed = current_speed;
  debug_msg.target_steering_angle = target_steering_angle;
  debug_msg.current_steering_angle = current_sa;
  debug_msg.cmd_vel_x = cmd_vel_in.x;
  debug_msg.cmd_vel_y = cmd_vel_in.y;
  debug_msg.cmd_vel_tw = cmd_vel_in.tw;
  debug_msg.odom_vel_x = odom_vel.x;
  debug_msg.odom_vel_y = odom_vel.y;
  debug_msg.odom_vel_tw = odom_vel.tw;

  bool modified = false;
  Velocity result_vel = robot_action.req_vel;

  // Check if speed crosses zero (direction reversal)
  bool crosses_zero = (target_speed > 0 && current_speed < 0) ||
    (target_speed < 0 && current_speed > 0);
  debug_msg.crosses_zero = crosses_zero;

  if (crosses_zero) {
    if (std::abs(current_speed) > low_speed_threshold_) {
      // Must decelerate first — clamp tw to maintain current steering angle
      result_vel.tw = steeringAngleToTw(result_vel.x, current_sa);
      modified = true;
      debug_msg.steering_angle_limit = current_sa;
    }
    // else: abs(current) < threshold → allow steering freely
    if (modified) {
      robot_action.req_vel = result_vel;
      robot_action.polygon_name = polygon_name_;
      robot_action.action_type = LIMIT;
    }
    debug_msg.modified = modified;
    debug_msg.result_vel_x = result_vel.x;
    debug_msg.result_vel_y = result_vel.y;
    debug_msg.result_vel_tw = result_vel.tw;
    steering_debug_pub_->publish(debug_msg);
    return modified;
  }

  // Speed does NOT cross zero
  // 1. Both abs(target) and abs(current) below threshold → done
  //    Use steering wheel speed (accounts for angular velocity) instead of baselink linear speed
  double target_sw_speed = baselinkToSteeringSpeed(cmd_vel_in.x, cmd_vel_in.tw);
  double current_sw_speed = baselinkToSteeringSpeed(odom_vel.x, odom_vel.tw);
  debug_msg.target_sw_speed = target_sw_speed;
  debug_msg.current_sw_speed = current_sw_speed;

  bool both_below = std::abs(target_sw_speed) < low_speed_threshold_ &&
    std::abs(current_sw_speed) < low_speed_threshold_;
  debug_msg.both_below_threshold = both_below;
  if (both_below) {
    debug_msg.modified = false;
    debug_msg.result_vel_x = result_vel.x;
    debug_msg.result_vel_y = result_vel.y;
    debug_msg.result_vel_tw = result_vel.tw;
    steering_debug_pub_->publish(debug_msg);
    return false;
  }

  const SubPolygonParameter * current_field = findField(current_sw_speed, current_sa);
  debug_msg.current_field_name = current_field ? current_field->velocity_polygon_name_ : "";


  // 2. Check if target angle is in same bucket (angle range) as current
  bool same_bucket = current_field != nullptr &&
    target_steering_angle >= current_field->steering_angle_min_ &&
    target_steering_angle <= current_field->steering_angle_max_;
  debug_msg.same_bucket = same_bucket;

  if (same_bucket) {
    // If result velocity falls into some faster field (same bucket), check the
    // one-step-faster field for collision. If in collision, limit to current field.
    double result_sw_speed = baselinkToSteeringSpeed(result_vel.x, result_vel.tw);
    const SubPolygonParameter * result_field = findField(result_sw_speed, target_steering_angle);
    if (result_field != nullptr && result_field != current_field &&
      std::abs(result_sw_speed) > std::abs(current_sw_speed))
    {
      bool forward = current_sw_speed >= 0;
      auto fields_at_angle = findFieldsForAngle(target_steering_angle, forward);
      for (size_t i = 0; i < fields_at_angle.size(); i++) {
        if (fields_at_angle[i] == current_field && i + 1 < fields_at_angle.size()) {
          const SubPolygonParameter * next_field = fields_at_angle[i + 1];
          debug_msg.next_field_name = next_field->velocity_polygon_name_;
          int pts = getPointsInsideSubPolygon(*next_field, collision_points_map);
          debug_msg.next_field_collision_pts = pts;
          if (pts >= min_points_) {
            // Forward: limit to linear_max (upper bound)
            // Backward: limit to linear_min (lower bound, more negative = faster)
            double limit_sw = (current_sw_speed >= 0) ?
              current_field->linear_max_ : current_field->linear_min_;
            double max_baselink = steeringToBaselinkSpeed(limit_sw, current_sa);
            if (std::abs(result_vel.x) > std::abs(max_baselink)) {
              debug_msg.speed_limit_applied = max_baselink;
              result_vel.x = max_baselink;
              result_vel.tw = steeringAngleToTw(result_vel.x, target_steering_angle);
              modified = true;
            }
          }
          break;
        }
      }
    }
    // Same bucket → done
    if (modified) {
      robot_action.req_vel = result_vel;
      robot_action.polygon_name = polygon_name_;
      robot_action.action_type = LIMIT;
    }
    debug_msg.modified = modified;
    debug_msg.result_vel_x = result_vel.x;
    debug_msg.result_vel_y = result_vel.y;
    debug_msg.result_vel_tw = result_vel.tw;
    steering_debug_pub_->publish(debug_msg);
    return modified;
  }

  // 3. Different bucket — find neighbouring bucket (one step in steering direction)

  double neighbour_angle;
  if (target_steering_angle > current_sa) {
    neighbour_angle = current_field->steering_angle_max_;
  } else {
    neighbour_angle = current_field->steering_angle_min_;
  }
  // Step just past the boundary to land in the neighbouring bucket
  constexpr double kAngleEps = 0.01;
  double lookup_angle = (target_steering_angle > current_sa) ?
    neighbour_angle + kAngleEps : neighbour_angle - kAngleEps;

  bool forward = target_sw_speed >= 0;
  auto neighbour_fields = findFieldsForAngle(lookup_angle, forward);
  if (neighbour_fields.empty()) {
    debug_msg.modified = false;
    debug_msg.result_vel_x = result_vel.x;
    debug_msg.result_vel_y = result_vel.y;
    debug_msg.result_vel_tw = result_vel.tw;
    steering_debug_pub_->publish(debug_msg);
    return false;
  }

  // Find the fastest field that covers max(|current|, |target|) speed
  double max_sw_speed = std::max(std::abs(current_sw_speed), std::abs(target_sw_speed));

  // Find the valid field: start from fastest covering field, walk down
  const SubPolygonParameter * valid_field = nullptr;
  int start_idx = static_cast<int>(neighbour_fields.size()) - 1;

  // Find the starting field (fastest that covers max_sw_speed)
  for (int i = start_idx; i >= 0; i--) {
    if (max_sw_speed >= std::abs(neighbour_fields[i]->linear_min_) &&
      max_sw_speed <= std::abs(neighbour_fields[i]->linear_max_))
    {
      start_idx = i;
      break;
    }
    // If max_sw_speed is beyond all fields, start from the fastest
    if (i == 0) {
      start_idx = static_cast<int>(neighbour_fields.size()) - 1;
    }
  }

  // Determine valid field: fastest collision-free field in neighbor bucket
  int start_pts = getPointsInsideSubPolygon(
    *neighbour_fields[start_idx], collision_points_map);
  debug_msg.neighbour_collision_pts = start_pts;
  if (start_pts < min_points_) {
    // Fastest field is collision-free — use it as valid field.
    // Speed must still be limited to this field's max to prevent field exceedance
    // when entering a bucket with lower max speed (e.g. straight → turned).
    valid_field = neighbour_fields[start_idx];
  } else {
    // Walk down speed fields until a collision-free one is found
    for (int i = start_idx; i >= 0; i--) {
      if (getPointsInsideSubPolygon(*neighbour_fields[i], collision_points_map) < min_points_) {
        valid_field = neighbour_fields[i];
        break;
      }
    }

    if (valid_field == nullptr) {
      // All fields in collision — use the slowest field in the target direction
      // (allowed even if in collision)
      valid_field = neighbour_fields[0];  // sorted ascending, index 0 is slowest
    }
  }
  debug_msg.valid_field_name = valid_field->velocity_polygon_name_;

  // 4. Adapt speed and steering angle
  // 4a. Limit target speed to valid field's speed boundary (converted to baselink)
  // Forward: limit to linear_max; Backward: limit to linear_min
  double valid_limit_sw = (target_sw_speed >= 0) ?
    valid_field->linear_max_ : valid_field->linear_min_;
  double valid_max_baselink = steeringToBaselinkSpeed(
    valid_limit_sw, neighbour_angle);
  if (std::abs(result_vel.x) > std::abs(valid_max_baselink)) {
    debug_msg.speed_limit_applied = valid_max_baselink;
    result_vel.x = valid_max_baselink;
    modified = true;
  }

  // 4b. Only limit steering angle if current speed is larger than max valid speed
  double current_baselink_abs = std::abs(current_speed);
  double valid_max_baselink_abs = std::abs(valid_max_baselink);
  if (current_baselink_abs > valid_max_baselink_abs && current_field != nullptr) {
    double limited_sa;
    if (target_steering_angle > current_sa) {
      limited_sa = current_field->steering_angle_max_;
    } else {
      limited_sa = current_field->steering_angle_min_;
    }
    debug_msg.steering_angle_limit = limited_sa;
    result_vel.tw = steeringAngleToTw(result_vel.x, limited_sa);
    modified = true;
  }

  if (modified) {
    robot_action.req_vel = result_vel;
    robot_action.polygon_name = polygon_name_;
    robot_action.action_type = LIMIT;
  }

  debug_msg.modified = modified;
  debug_msg.result_vel_x = result_vel.x;
  debug_msg.result_vel_y = result_vel.y;
  debug_msg.result_vel_tw = result_vel.tw;
  steering_debug_pub_->publish(debug_msg);
  return modified;
}

bool VelocityPolygon::clampToMaxField(
  const Velocity & odom_vel, Action & robot_action)
{
  // Only applies to steering-angle-based velocity polygons
  if (sub_polygons_.empty() || !sub_polygons_[0].use_steering_angle_) {
    return false;
  }

  // Physical steering angle from odometry
  double physical_sa = computeSteeringAngle(odom_vel);

  // Direction from commanded velocity
  double cmd_x = robot_action.req_vel.x;
  bool forward = cmd_x >= 0;

  // Find all fields at the physical steering angle for the commanded direction
  auto fields = findFieldsForAngle(physical_sa, forward);

  if (fields.empty()) {
    // No fields at this angle — if velocity is non-zero, zero it
    if (std::abs(cmd_x) > 1e-6) {
      RCLCPP_INFO(
        logger_,
        "[%s] clampToMaxField: no fields at physical_sa=%.3f, zeroing velocity",
        polygon_name_.c_str(), physical_sa);
      robot_action.req_vel.x = 0.0;
      robot_action.req_vel.y = 0.0;
      robot_action.req_vel.tw = 0.0;
      robot_action.polygon_name = polygon_name_;
      robot_action.action_type = LIMIT;
      return true;
    }
    return false;
  }

  // Fastest field is the last one (sorted slowest first)
  const SubPolygonParameter * fastest = fields.back();

  // Max steering wheel speed for this field
  double max_sw = forward ? fastest->linear_max_ : fastest->linear_min_;

  // Compute commanded steering angle and steering wheel speed
  double cmd_sa = computeSteeringAngle(robot_action.req_vel);
  double cmd_sw = baselinkToSteeringSpeed(robot_action.req_vel.x, robot_action.req_vel.tw);

  // Check if commanded sw speed exceeds max
  if (std::abs(cmd_sw) > std::abs(max_sw)) {
    double clamped_baselink = steeringToBaselinkSpeed(max_sw, cmd_sa);
    RCLCPP_INFO(
      logger_,
      "[%s] clampToMaxField: cmd_sw=%.3f exceeds max_sw=%.3f at physical_sa=%.3f, "
      "clamping vel.x from %.3f to %.3f",
      polygon_name_.c_str(), cmd_sw, max_sw, physical_sa,
      robot_action.req_vel.x, clamped_baselink);
    robot_action.req_vel.x = clamped_baselink;
    robot_action.req_vel.tw = steeringAngleToTw(clamped_baselink, cmd_sa);
    robot_action.polygon_name = polygon_name_;
    robot_action.action_type = LIMIT;
    return true;
  }

  return false;
}

}  // namespace nav2_collision_monitor
