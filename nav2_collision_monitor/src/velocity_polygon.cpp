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

    nav2_util::declare_parameter_if_not_declared(
      node, polygon_name_ + ".wheelbase", rclcpp::ParameterValue(1.0));
    wheelbase_ = node->get_parameter(polygon_name_ + ".wheelbase").as_double();

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
      linear_limit_ = sub_polygon.linear_limit_;
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
  // 1. Check angular/steering range first
  bool in_range;
  if (sub_polygon.use_steering_angle_) {
    if (std::abs(cmd_vel_in.x) < 1e-6) {
      current_steering_angle_ = (std::abs(cmd_vel_in.tw) < 1e-6) ?
                               0.0 :
                               (cmd_vel_in.tw > 0 ? M_PI / 2 : -M_PI / 2);
    } else {
      double angular_vel = cmd_vel_in.tw;
      if (cmd_vel_in.x < 0) {
        angular_vel = -angular_vel;
      }

      current_steering_angle_ = std::atan2(wheelbase_ * angular_vel, std::abs(cmd_vel_in.x));
    }

    RCLCPP_DEBUG(
      logger_,
      "Calculated steering angle: %.2f (limits: %.2f to %.2f), linear_vel: %.2f, angular_vel: %.2f",
      current_steering_angle_,
      sub_polygon.steering_angle_min_,
      sub_polygon.steering_angle_max_,
      cmd_vel_in.x,
      cmd_vel_in.tw
    );

    in_range = current_steering_angle_ <= sub_polygon.steering_angle_max_ &&
               current_steering_angle_ >= sub_polygon.steering_angle_min_;
  } else {
    in_range =
      (cmd_vel_in.tw <= sub_polygon.theta_max_ &&
      cmd_vel_in.tw >= sub_polygon.theta_min_);
  }

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

}  // namespace nav2_collision_monitor
