// Copyright (c) 2023 Pixel Robotics GmbH
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

#include "nav2_collision_monitor/polygon_source.hpp"

#include <algorithm>
#include <cmath>
#include <functional>
#include <string>

#include "geometry_msgs/msg/polygon_stamped.hpp"
#include "tf2/transform_datatypes.hpp"

#include "nav2_ros_common/node_utils.hpp"
#include "nav2_util/robot_utils.hpp"


namespace nav2_collision_monitor
{

PolygonSource::PolygonSource(
  const nav2::LifecycleNode::WeakPtr & node,
  const std::string & source_name,
  const std::shared_ptr<tf2_ros::Buffer> tf_buffer,
  const std::string & base_frame_id,
  const std::string & global_frame_id,
  const tf2::Duration & transform_tolerance,
  const rclcpp::Duration & source_timeout,
  const bool base_shift_correction)
: Source(
    node, source_name, tf_buffer, base_frame_id, global_frame_id,
    transform_tolerance, source_timeout, base_shift_correction)
{
}

PolygonSource::~PolygonSource()
{
  data_sub_.reset();
}

void PolygonSource::configure()
{
  Source::configure();
  auto node = node_.lock();
  if (!node) {
    throw std::runtime_error{"Failed to lock node"};
  }

  std::string source_topic;

  getParameters(source_topic);

  data_sub_ = node->create_subscription<geometry_msgs::msg::PolygonInstanceStamped>(
    source_topic,
    std::bind(&PolygonSource::dataCallback, this, std::placeholders::_1),
    nav2::qos::SensorDataQoS());
}

bool PolygonSource::getData(
  const rclcpp::Time & curr_time,
  std::vector<Point> & data)
{
  // Remove stale data first, so a source whose polygons all aged out is
  // detected as empty below
  if (source_timeout_.seconds() != 0.0){
    data_.erase(
      std::remove_if(
        data_.begin(), data_.end(),
        [this, curr_time](const geometry_msgs::msg::PolygonInstanceStamped & polygon_stamped) {
          return curr_time - rclcpp::Time(polygon_stamped.header.stamp) > source_timeout_;
        }), data_.end());
  }

  // Ignore data from the source if it is not being published yet or
  // not published for a long time
  if (data_.empty()) {
    if (treat_empty_as_valid_) {
      return true;
    }
    RCLCPP_WARN_THROTTLE(
      logger_, *clock_, 2000,
      "[%s]: No polygon source data (none received yet or all polygons older than "
      "source_timeout)", source_name_.c_str());
    return false;
  }

  tf2::Stamped<tf2::Transform> tf_transform;
  // Frame tf_transform was looked up for. Without base shift correction the transform only
  // depends on the frame, which the polygons of a source typically share.
  std::string tf_frame_id;
  for (const auto & polygon_instance : data_) {
    if (polygon_instance.polygon.polygon.points.empty()) {
      // Publishers clear a polygon id by sending it without points
      continue;
    }
    if (base_shift_correction_) {
      // Obtaining the transform to get data from source frame and time where it was received
      // to the base frame and current time
      if (
        !nav2_util::getTransform(
          polygon_instance.header.frame_id, polygon_instance.header.stamp,
          base_frame_id_, curr_time, global_frame_id_,
          transform_tolerance_, tf_buffer_, tf_transform))
      {
        return false;
      }
    } else {
      // Obtaining the transform to get data from source frame to base frame without time shift
      // considered. Less accurate but much more faster option not dependent on state estimation
      // frames.
      if (polygon_instance.header.frame_id != tf_frame_id) {
        if (
          !nav2_util::getTransform(
            polygon_instance.header.frame_id, base_frame_id_,
            transform_tolerance_, tf_buffer_, tf_transform))
        {
          return false;
        }
        tf_frame_id = polygon_instance.header.frame_id;
      }
    }
    geometry_msgs::msg::PolygonStamped poly_out, polygon_stamped;
    geometry_msgs::msg::TransformStamped tf = tf2::toMsg(tf_transform);
    polygon_stamped.header = polygon_instance.header;
    polygon_stamped.polygon = polygon_instance.polygon.polygon;
    tf2::doTransform(polygon_stamped, poly_out, tf);
    convertPolygonStampedToPoints(poly_out, data);
  }
  return true;
}

void PolygonSource::convertPolygonStampedToPoints(
  const geometry_msgs::msg::PolygonStamped & polygon, std::vector<Point> & data) const
{
  // Iterate over the vertices of the polygon
  for (size_t i = 0; i < polygon.polygon.points.size(); ++i) {
    const auto & current_point = polygon.polygon.points[i];
    const auto & next_point =
      polygon.polygon.points[(i + 1) % polygon.polygon.points.size()];

    const double x0 = current_point.x;
    const double y0 = current_point.y;
    const double edge_x = next_point.x - x0;
    const double edge_y = next_point.y - y0;

    // Calculate the number of points to sample in the current segment
    const double segment_length = std::hypot(edge_x, edge_y);
    const int num_points_in_segment =
      std::max(static_cast<int>(std::ceil(segment_length / sampling_distance_)), 1);

    // Part [t_min, t_max] of the segment (0 = current_point, 1 = next_point) inside the
    // max_range_ square (Liang-Barsky clipping)
    double t_min = 0.0;
    double t_max = 1.0;
    if (max_range_ > 0.0) {
      const double p[4] = {-edge_x, edge_x, -edge_y, edge_y};
      const double q[4] = {x0 + max_range_, max_range_ - x0, y0 + max_range_, max_range_ - y0};
      for (int k = 0; k < 4 && t_min <= t_max; ++k) {
        if (p[k] == 0.0) {
          if (q[k] < 0.0) {
            t_max = -1.0;  // parallel to this side of the square and outside of it
          }
        } else if (p[k] < 0.0) {
          t_min = std::max(t_min, q[k] / p[k]);
        } else {
          t_max = std::min(t_max, q[k] / p[k]);
        }
      }
      if (t_min > t_max) {
        continue;
      }
    }

    // Calculate the step size for each pair of vertices
    const double dx = edge_x / num_points_in_segment;
    const double dy = edge_y / num_points_in_segment;

    // Sample the points with equal spacing. The samples stay anchored at current_point, so
    // clipping only drops points and never moves them. One sample of slack on both ends
    // keeps rounding at the square border from dropping a point inside of it.
    const int j_min = std::max(
      static_cast<int>(std::floor(t_min * num_points_in_segment)) - 1, 0);
    const int j_max = std::min(
      static_cast<int>(std::ceil(t_max * num_points_in_segment)) + 1, num_points_in_segment);
    for (int j = j_min; j <= j_max; ++j) {
      Point p;
      p.x = x0 + j * dx;
      p.y = y0 + j * dy;
      data.push_back(p);
    }
  }
}

void PolygonSource::getParameters(std::string & source_topic)
{
  auto node = node_.lock();
  if (!node) {
    throw std::runtime_error{"Failed to lock node"};
  }

  getCommonParameters(source_topic);

  sampling_distance_ = node->declare_or_get_parameter(
    source_name_ + ".sampling_distance", 0.1);
  max_range_ = node->declare_or_get_parameter(
    source_name_ + ".max_range", 0.0);
  treat_empty_as_valid_ = node->declare_or_get_parameter(
    source_name_ + ".treat_empty_as_valid", false);
}

void PolygonSource::dataCallback(geometry_msgs::msg::PolygonInstanceStamped::ConstSharedPtr msg)
{
  auto node = node_.lock();
  if (!node) {
    throw std::runtime_error{"Failed to lock node"};
  }
  auto curr_time = node->now();

  // check if older similar polygon exists already and replace it with the new one
  for (auto & polygon_stamped : data_) {
    if (msg->polygon.id == polygon_stamped.polygon.id) {
      polygon_stamped = *msg;
      return;
    }
  }
  data_.push_back(*msg);
}

}  // namespace nav2_collision_monitor
