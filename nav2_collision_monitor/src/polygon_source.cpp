// Copyright (c) 2022 Samsung R&D Institute Russia
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

#include <functional>

#include "geometry_msgs/msg/polygon_stamped.hpp"

#include "nav2_util/node_utils.hpp"
#include "nav2_util/robot_utils.hpp"

using rcl_interfaces::msg::ParameterType;

namespace nav2_collision_monitor
{

PolygonSource::PolygonSource(
  const nav2_util::LifecycleNode::WeakPtr & node,
  const std::string & source_name,
  const std::shared_ptr<tf2_ros::Buffer> tf_buffer,
  const std::string & base_frame_id,
  const std::string & global_frame_id,
  const tf2::Duration & transform_tolerance,
  const rclcpp::Duration & source_timeout,
  const bool base_shift_correction)
: Source(
    node, source_name, tf_buffer, base_frame_id, global_frame_id,
    transform_tolerance, source_timeout, base_shift_correction),
  data_(nullptr)
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

  rclcpp::QoS qos = rclcpp::SensorDataQoS();  // set to default
  data_sub_ = node->create_subscription<nav2_msgs::msg::PolygonsArray>(
    source_topic, qos,
    std::bind(&PolygonSource::dataCallback, this, std::placeholders::_1));
}

void PolygonSource::getData(
  const rclcpp::Time & curr_time,
  std::vector<Point> & data) const
{
  // Ignore data from the source if it is not being published yet or
  // not published for a long time
  if (data_ == nullptr || data_->polygons.empty()) {
    return;
  }
  // get the earliest time stamp from the polygon array
  // TODO: refine
  rclcpp::Time earliest_stamp = rclcpp::Time(data_->polygons[0].header.stamp);
  for (const auto& polygon : data_->polygons) {
    if (rclcpp::Time(polygon.header.stamp) < earliest_stamp) {
      earliest_stamp = rclcpp::Time(polygon.header.stamp);
    }
  }
  if (!sourceValid(earliest_stamp, curr_time)) {
    return;
  }

  geometry_msgs::msg::TransformStamped tf;
  if (base_shift_correction_) {
    // Obtaining the transform to get data from source frame and time where it was received
    // to the base frame and current time
    if (
      !nav2_util::getTransform(
        data_->header.frame_id, data_->header.stamp,
        base_frame_id_, curr_time, global_frame_id_,
        transform_tolerance_, tf_buffer_, tf))
    {
      return;
    }
  } else {
    // Obtaining the transform to get data from source frame to base frame without time shift
    // considered. Less accurate but much more faster option not dependent on state estimation
    // frames.
    if (
      !nav2_util::getTransform(
        data_->header.frame_id, base_frame_id_,
        transform_tolerance_, tf_buffer_, tf))
    {
      return;
    }
  }

  for (const auto& polygon : data_->polygons) {
    geometry_msgs::msg::PolygonStamped poly_out;
    tf2::doTransform(polygon, poly_out, tf);
    convertPolygonStampedToPoints(poly_out, data);
  }

}
void PolygonSource::convertPolygonStampedToPoints(const geometry_msgs::msg::PolygonStamped& polygon, std::vector<Point>& data) const
{
    // Calculate the total perimeter of the polygon
    double perimeter = 0.0;
    double spacing = 0.1;
    for (size_t i = 0; i < polygon.polygon.points.size(); ++i) {
        const auto& currentPoint = polygon.polygon.points[i];
        const auto& nextPoint = polygon.polygon.points[(i + 1) % polygon.polygon.points.size()];
        perimeter += sqrt(pow(nextPoint.x - currentPoint.x, 2) + pow(nextPoint.y - currentPoint.y, 2));
    }

    // Iterate over the vertices of the polygon
    for (size_t i = 0; i < polygon.polygon.points.size(); ++i) {
        const auto& currentPoint = polygon.polygon.points[i];
        const auto& nextPoint = polygon.polygon.points[(i + 1) % polygon.polygon.points.size()];

        // Calculate the distance between the current and next points
        double segmentLength = sqrt(pow(nextPoint.x - currentPoint.x, 2) + pow(nextPoint.y - currentPoint.y, 2));

        // Calculate the number of points to sample in the current segment
        size_t numPointsInSegment = std::max(static_cast<size_t>(segmentLength / spacing), static_cast<size_t>(1));

        // Calculate the step size for each pair of vertices
        const double dx = (nextPoint.x - currentPoint.x) / numPointsInSegment;
        const double dy = (nextPoint.y - currentPoint.y) / numPointsInSegment;

        // Sample the points with equal spacing
        for (size_t j = 0; j <= numPointsInSegment; ++j) {
            Point p;
            p.x = currentPoint.x + j * dx;
            p.y = currentPoint.y + j * dy;
            data.push_back(p);
        }
    }
}

void PolygonSource::getParameters(std::string & source_topic)
{
  getCommonParameters(source_topic);
}

void PolygonSource::dataCallback(nav2_msgs::msg::PolygonsArray::ConstSharedPtr msg)
{
  data_ = msg;
}

}  // namespace nav2_collision_monitor
