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
#include "nav2_util/geometry_utils.hpp"

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
    RCLCPP_INFO(
      logger_, "[%s]: Using wheelbase: %.4f m", polygon_name_.c_str(), wheelbase_);
    if (std::abs(wheelbase_ - 1.0) < 1e-6) {
      RCLCPP_WARN(
        logger_,
        "[%s]: Wheelbase is default (1.0 m). Set '%s.wheelbase' to the robot's "
        "actual wheelbase to ensure correct steering wheel speed calculations.",
        polygon_name_.c_str(), polygon_name_.c_str());
    }

    low_speed_threshold_ = node->declare_or_get_parameter(
      polygon_name_ + ".low_speed_threshold", 0.1);
    speed_margin_ = node->declare_or_get_parameter(
      polygon_name_ + ".speed_margin", 0.02);
    angle_margin_ = node->declare_or_get_parameter(
      polygon_name_ + ".angle_margin", 0.01);

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

      // Parse modes list - defaults to ["default"] if not specified
      std::vector<std::string> modes;
      try {
        modes = node->declare_or_get_parameter<std::vector<std::string>>(
          polygon_name_ + "." + velocity_polygon_name + ".modes");
      } catch (const rclcpp::exceptions::ParameterNotDeclaredException &) {
        modes = {"default"};
      } catch (const rclcpp::exceptions::ParameterUninitializedException &) {
        modes = {"default"};
      } catch (const rclcpp::exceptions::InvalidParameterValueException &) {
        modes = {"default"};
      }

      if (!modes.empty()) {
        std::string modes_str;
        for (const auto & m : modes) {
          if (!modes_str.empty()) {modes_str += ", ";}
          modes_str += m;
        }
        RCLCPP_INFO(
          logger_, "[%s]: Sub-polygon %s active in modes: [%s]",
          polygon_name_.c_str(), velocity_polygon_name.c_str(), modes_str.c_str());
      }

      SubPolygonParameter sub_polygon = {
        poly, velocity_polygon_name, linear_min, linear_max, theta_min,
        theta_max, steering_angle_min, steering_angle_max, use_steering_angle,
        direction_end_angle, direction_start_angle,
        slowdown_ratio, linear_limit, angular_limit, time_before_collision,
        modes,
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

  next_field_poly_pub_ = node->create_publisher<geometry_msgs::msg::PolygonStamped>(
    "~/next_field_polygon", rclcpp::QoS(1));

  next_field_collision_points_pub_ =
    node->create_publisher<visualization_msgs::msg::MarkerArray>(
    "~/next_field_collision_points_marker", rclcpp::QoS(1));

  return true;
}

void VelocityPolygon::setFieldsMode(const std::string & mode)
{
  if (current_fields_mode_ != mode) {
    RCLCPP_INFO(
      logger_, "[%s]: Fields mode changed from '%s' to '%s'",
      polygon_name_.c_str(), current_fields_mode_.c_str(), mode.c_str());
    current_fields_mode_ = mode;
  }
}

std::string VelocityPolygon::getFieldsMode() const
{
  return current_fields_mode_;
}

bool VelocityPolygon::isSubPolygonActiveInCurrentMode(
  const SubPolygonParameter & sub_polygon) const
{
  if (sub_polygon.modes_.empty()) {
    return true;
  }
  return std::find(
    sub_polygon.modes_.begin(), sub_polygon.modes_.end(),
    current_fields_mode_) != sub_polygon.modes_.end();
}

void VelocityPolygon::updatePolygon(const Velocity & cmd_vel_in)
{
  for (auto & sub_polygon : sub_polygons_) {
    if (!isSubPolygonActiveInCurrentMode(sub_polygon)) {
      continue;
    }
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

  // Non-steering-angle mode: check angular range first
  bool in_range = cmd_vel_in.tw <= sub_polygon.theta_max_ &&
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
    if (!isSubPolygonActiveInCurrentMode(sp)) {
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
    if (!isSubPolygonActiveInCurrentMode(sp)) {
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
  return nav2_util::geometry_utils::isPointInsidePolygon(point.x, point.y, vertices);
}

int VelocityPolygon::getPointsInsideSubPolygon(
  const SubPolygonParameter & sub_polygon,
  const std::unordered_map<std::string, std::vector<Point>> & collision_points_map,
  std::unordered_map<std::string, std::vector<Point>> * points_per_source_out) const
{
  int num = 0;
  std::vector<std::string> polygon_sources_names = getSourcesNames();

  for (const auto & source_name : polygon_sources_names) {
    const auto & iter = collision_points_map.find(source_name);
    if (iter != collision_points_map.end()) {
      for (const auto & point : iter->second) {
        if (isPointInsidePoly(point, sub_polygon.poly_)) {
          num++;
          if (points_per_source_out != nullptr) {
            (*points_per_source_out)[source_name].push_back(point);
          }
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

  // Track the field being checked for obstacles (published for visualization)
  const SubPolygonParameter * checked_field = nullptr;
  // Collision points inside the next_field, grouped by source — populated only
  // when the marker topic has subscribers (cheap when nobody listens).
  std::unordered_map<std::string, std::vector<Point>> next_field_points_by_source;
  const bool publish_next_field_points =
    next_field_collision_points_pub_->get_subscription_count() > 0;

  nav2_msgs::msg::SteeringValidationDebug debug_msg;
  debug_msg.header.stamp = clock_->now();
  debug_msg.polygon_name = polygon_name_;
  debug_msg.steering_angle_limit = std::numeric_limits<float>::quiet_NaN();
  debug_msg.speed_limit_applied = 0.0;
  debug_msg.final_sw = 0.0;
  debug_msg.limited_sa = std::numeric_limits<float>::quiet_NaN();
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

  const double target_speed = cmd_vel_in.x;
  const double current_speed = odom_vel.x;

  const double target_sa = computeSteeringAngle(cmd_vel_in);
  const double current_sa = computeSteeringAngle(odom_vel);

  debug_msg.target_speed = target_speed;
  debug_msg.current_speed = current_speed;
  debug_msg.target_steering_angle = target_sa;
  debug_msg.current_steering_angle = current_sa;
  debug_msg.cmd_vel_x = cmd_vel_in.x;
  debug_msg.cmd_vel_y = cmd_vel_in.y;
  debug_msg.cmd_vel_tw = cmd_vel_in.tw;
  debug_msg.odom_vel_x = odom_vel.x;
  debug_msg.odom_vel_y = odom_vel.y;
  debug_msg.odom_vel_tw = odom_vel.tw;

  // --- Work in steering wheel domain ---
  // Convert the step-1 result to steering wheel speed + steering angle.
  // All limits below modify these two variables; conversion back to
  // baselink happens only at the very end via swToBaselink().
  double result_sw = baselinkToSteeringSpeed(robot_action.req_vel.x, robot_action.req_vel.tw);
  double result_sa = target_sa;

  // Utility: convert steering wheel (speed, angle) back to baselink Velocity
  auto swToBaselink = [&](double sw, double sa) -> Velocity {
    Velocity v;
    v.x = steeringToBaselinkSpeed(sw, sa);
    v.y = robot_action.req_vel.y;
    v.tw = steeringAngleToTw(v.x, sa);
    return v;
  };

  // Helper: convert result_sw/result_sa to baselink, publish debug + polygon, return
  auto apply_and_return = [&](bool mod) -> bool {
    Velocity result_vel;
    if (mod) {
      result_vel = swToBaselink(result_sw, result_sa);
      debug_msg.final_sw = result_sw;
      debug_msg.limited_sa = result_sa;
      debug_msg.speed_limit_applied = result_vel.x;
      robot_action.req_vel = result_vel;
      robot_action.polygon_name = polygon_name_;
      robot_action.action_type = LIMIT;
    } else {
      result_vel = robot_action.req_vel;
    }
    debug_msg.modified = mod;
    debug_msg.result_vel_x = result_vel.x;
    debug_msg.result_vel_y = result_vel.y;
    debug_msg.result_vel_tw = result_vel.tw;
    steering_debug_pub_->publish(debug_msg);
    if (checked_field != nullptr && next_field_poly_pub_->get_subscription_count() > 0) {
      geometry_msgs::msg::PolygonStamped poly_msg;
      poly_msg.header.frame_id = base_frame_id_;
      poly_msg.header.stamp = clock_->now();
      for (const auto & p : checked_field->poly_) {
        geometry_msgs::msg::Point32 pt;
        pt.x = p.x;
        pt.y = p.y;
        poly_msg.polygon.points.push_back(pt);
      }
      next_field_poly_pub_->publish(poly_msg);
    }
    if (publish_next_field_points &&
      next_field_collision_points_pub_->get_subscription_count() > 0)
    {
      visualization_msgs::msg::MarkerArray marker_array;
      int marker_id = 0;
      for (const auto & kv : next_field_points_by_source) {
        const std::string & source_name = kv.first;
        const std::vector<Point> & pts_vec = kv.second;
        visualization_msgs::msg::Marker m;
        m.header.frame_id = base_frame_id_;
        m.header.stamp = clock_->now();
        m.ns = "next_field_collision_points_" + source_name;
        m.id = marker_id++;
        m.type = visualization_msgs::msg::Marker::POINTS;
        m.action = visualization_msgs::msg::Marker::ADD;
        m.scale.x = 0.04;
        m.scale.y = 0.04;
        m.color.r = 1.0;
        m.color.g = 1.0;
        m.color.a = 1.0;
        m.lifetime = rclcpp::Duration(0, 0);
        m.frame_locked = true;
        for (const auto & p : pts_vec) {
          geometry_msgs::msg::Point gp;
          gp.x = p.x;
          gp.y = p.y;
          gp.z = 0.0;
          m.points.push_back(gp);
        }
        marker_array.markers.push_back(m);
      }
      next_field_collision_points_pub_->publish(std::move(marker_array));
    }
    return mod;
  };

  // --- Direction reversal check (uses baselink speed for direction) ---
  bool crosses_zero = (target_speed > 0 && current_speed < 0) ||
    (target_speed < 0 && current_speed > 0);
  debug_msg.crosses_zero = crosses_zero;

  if (crosses_zero) {
    if (std::abs(current_speed) > low_speed_threshold_) {
      // Must decelerate first — lock steering angle to current
      result_sa = current_sa;
      debug_msg.steering_angle_limit = current_sa;
      return apply_and_return(true);
    }
    // abs(current) < threshold → allow steering freely
    return apply_and_return(false);
  }

  // --- Steering wheel speeds ---
  double target_sw = baselinkToSteeringSpeed(cmd_vel_in.x, cmd_vel_in.tw);
  double current_sw = baselinkToSteeringSpeed(odom_vel.x, odom_vel.tw);
  debug_msg.target_sw_speed = target_sw;
  debug_msg.current_sw_speed = current_sw;

  // Step 1: Both abs(target) and abs(current) below threshold → allow free steering
  bool both_below = std::abs(target_sw) < low_speed_threshold_ &&
    std::abs(current_sw) < low_speed_threshold_;
  debug_msg.both_below_threshold = both_below;
  if (both_below) {
    return apply_and_return(false);
  }

  // --- Find current field ---
  const SubPolygonParameter * current_field = findField(current_sw, current_sa);
  debug_msg.current_field_name = current_field ? current_field->velocity_polygon_name_ : "";

  if (current_field == nullptr) {
    RCLCPP_WARN(
      logger_,
      "[%s] validateSteering: no field matches current velocity. "
      "odom=(%.3f, %.3f, %.3f), sw=%.3f, sa=%.3f, "
      "cmd=(%.3f, %.3f, %.3f), target_sw=%.3f, target_sa=%.3f.",
      polygon_name_.c_str(),
      odom_vel.x, odom_vel.y, odom_vel.tw, current_sw, current_sa,
      cmd_vel_in.x, cmd_vel_in.y, cmd_vel_in.tw, target_sw, target_sa);
    return apply_and_return(false);
  }

  // --- Step 2: Same-bucket speed limit (always enforced) ---
  // Check one field up from the current field at the current physical angle:
  //  - Next field collision-free → allow up to next field's max
  //  - Next field has obstacles or doesn't exist → stay at current field's max
  bool forward_current = current_sw >= 0;
  auto fields_at_current_angle = findFieldsForAngle(current_sa, forward_current);
  double current_bucket_limit_sw = forward_current ?
    current_field->linear_max_ - speed_margin_ :
    current_field->linear_min_ + speed_margin_;
  debug_msg.next_field_name = "no field found above";

  for (size_t i = 0; i < fields_at_current_angle.size(); i++) {
    if (fields_at_current_angle[i] != current_field) {
      continue;
    }
    if (i + 1 < fields_at_current_angle.size()) {
      const SubPolygonParameter * next_field = fields_at_current_angle[i + 1];
      checked_field = next_field;
      debug_msg.next_field_name = next_field->velocity_polygon_name_;
      int pts = getPointsInsideSubPolygon(
        *next_field, collision_points_map,
        publish_next_field_points ? &next_field_points_by_source : nullptr);
      debug_msg.next_field_collision_pts = pts;
      if (pts < min_points_) {
        current_bucket_limit_sw = forward_current ?
          next_field->linear_max_ - speed_margin_ :
          next_field->linear_min_ + speed_margin_;
      }
    }
    break;
  }

  // --- Step 3: Same bucket check ---
  bool same_bucket =
    target_sa >= current_field->steering_angle_min_ &&
    target_sa <= current_field->steering_angle_max_;
  debug_msg.same_bucket = same_bucket;

  // These two variables are refined through the algorithm and applied to
  // result_sw / result_sa at the end, before the single baselink conversion.
  double effective_limit_sw;
  double limited_sa;

  if (same_bucket) {
    // 3a. At standstill the current field is not meaningful for speed limiting —
    // the robot must be free to start moving.
    if (std::abs(current_sw) < low_speed_threshold_) {
      return apply_and_return(false);
    }

    // 3b. Limit speed to current bucket's speed limit from step 2
    effective_limit_sw = current_bucket_limit_sw;
    limited_sa = target_sa;
  } else {
    // --- Steps 4–5: Different bucket ---
    // Find neighbouring bucket (one step in steering direction)
    double neighbour_angle;
    if (target_sa > current_sa) {
      neighbour_angle = current_field->steering_angle_max_;
    } else {
      neighbour_angle = current_field->steering_angle_min_;
    }
    // Step just past the boundary to land in the neighbouring bucket
    constexpr double kAngleEps = 0.01;
    double lookup_angle = (target_sa > current_sa) ?
      neighbour_angle + kAngleEps : neighbour_angle - kAngleEps;

    bool forward = target_sw >= 0;
    auto neighbour_fields = findFieldsForAngle(lookup_angle, forward);
    if (neighbour_fields.empty()) {
      return apply_and_return(false);
    }

    // 5.1: Find starting field (fastest that covers max(|current|, |target|) speed)
    double max_sw = std::max(std::abs(current_sw), std::abs(target_sw));
    const SubPolygonParameter * valid_field = nullptr;
    int start_idx = static_cast<int>(neighbour_fields.size()) - 1;

    for (int i = start_idx; i >= 0; i--) {
      if (max_sw >= std::abs(neighbour_fields[i]->linear_min_) &&
        max_sw <= std::abs(neighbour_fields[i]->linear_max_))
      {
        start_idx = i;
        break;
      }
      // If speed is beyond all fields, start from the fastest
      if (i == 0) {
        start_idx = static_cast<int>(neighbour_fields.size()) - 1;
      }
    }

    // Walk down from starting field to find collision-free valid field
    int start_pts = getPointsInsideSubPolygon(
      *neighbour_fields[start_idx], collision_points_map);
    debug_msg.neighbour_collision_pts = start_pts;
    if (start_pts < min_points_) {
      valid_field = neighbour_fields[start_idx];
    } else {
      for (int i = start_idx; i >= 0; i--) {
        if (getPointsInsideSubPolygon(
            *neighbour_fields[i], collision_points_map) < min_points_)
        {
          valid_field = neighbour_fields[i];
          break;
        }
      }
      if (valid_field == nullptr) {
        // All fields in collision — use the slowest (allowed even if in collision)
        valid_field = neighbour_fields[0];
      }
      checked_field = valid_field;
    }
    debug_msg.valid_field_name = valid_field->velocity_polygon_name_;

    // 6a. effective_limit_sw = min(current_bucket_limit, valid_field_limit)
    // Both sides have speed_margin_ applied: current_bucket_limit_sw from step 2,
    // valid_limit_sw here.
    double valid_limit_sw = (target_sw >= 0) ?
      valid_field->linear_max_ : valid_field->linear_min_;
    double valid_limit_sw_margined = (target_sw >= 0) ?
      valid_limit_sw - speed_margin_ : valid_limit_sw + speed_margin_;
    effective_limit_sw =
      (std::abs(current_bucket_limit_sw) < std::abs(valid_limit_sw_margined)) ?
      current_bucket_limit_sw : valid_limit_sw_margined;

    // 6b. Limit steering angle — one bucket step at a time.
    // Margins (angle_margin_) keep the result slightly inside the field boundary
    // so the next cycle's findField lands in a real field.
    // If current speed exceeds the neighbour's max: stay at current bucket boundary.
    if (std::abs(current_sw) > std::abs(valid_limit_sw)) {
      // Too fast for neighbour — hold at current bucket boundary (inset by margin)
      limited_sa = (target_sa > current_sa) ?
        current_field->steering_angle_max_ - angle_margin_ :
        current_field->steering_angle_min_ + angle_margin_;
    } else {
      // Speed OK — use target angle
      limited_sa = target_sa;
    }
    if (std::abs(limited_sa - target_sa) > 1e-9) {
      debug_msg.steering_angle_limit = limited_sa;
    }
  }

  // --- Apply all limits in steering wheel domain ---
  bool speed_needs_limit = std::abs(result_sw) > std::abs(effective_limit_sw);
  bool angle_needs_limit = std::abs(limited_sa - target_sa) > 1e-9;
  if (speed_needs_limit) {
    result_sw = effective_limit_sw;
  }
  result_sa = limited_sa;

  // --- Convert to baselink only here, just before return ---
  return apply_and_return(speed_needs_limit || angle_needs_limit);
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

  // Max steering wheel speed for this field (inset by margin to stay inside)
  double max_sw = forward ?
    fastest->linear_max_ - speed_margin_ : fastest->linear_min_ + speed_margin_;

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
