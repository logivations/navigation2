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
  dyn_params_handler_.reset();
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
  dyn_params_handler_ = node->add_on_set_parameters_callback(
    std::bind(
      &PolygonSource::dynamicParametersCallback,
      this,
      std::placeholders::_1));


  rclcpp::QoS qos = rclcpp::SensorDataQoS();  // set to default
  data_sub_ = node->create_subscription<nav2_msgs::msg::PolygonsArray>(
    source_topic, qos,
    std::bind(&PolygonSource::dataCallback, this, std::placeholders::_1));
}

rcl_interfaces::msg::SetParametersResult
PolygonSource::dynamicParametersCallback(
  std::vector<rclcpp::Parameter> parameters)
{
  Source::dynamicParametersCallback(parameters);

  rcl_interfaces::msg::SetParametersResult result;

  // for (auto parameter : parameters) {
  //   const auto & param_type = parameter.get_type();
  //   const auto & param_name = parameter.get_name();

  //   if (param_type == ParameterType::PARAMETER_DOUBLE) {
  //     if (param_name == source_name_ + "." + "min_height") {
  //       min_height_ = parameter.as_double();
  //     } else if (param_name == source_name_ + "." + "max_height") {
  //       max_height_ = parameter.as_double();
  //     }
  //   }
  // }
  result.successful = true;
  return result;
}

void PolygonSource::getData(
  const rclcpp::Time & curr_time,
  std::vector<Point> & data) const
{
  // Ignore data from the source if it is not being published yet or
  // not published for a long time
  if (data_ == nullptr) {
    return;
  }
  // get the earliest time stamp from the polygon array
  // TODO: refine
  rclcpp::Time earliest_stamp = rclcpp::Time(data_->polygons[0].header.stamp);
  for (const auto& polygon : data_->polygons) {
    if (rclcpp::Time(polygon.header.stamp) < earliest_stamp) {
      earliest_stamp = polygon.header.stamp;
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
    convertPolygonStampedToVector(poly_out, data);
  }

}

void PolygonSource::convertPolygonStampedToVector(const geometry_msgs::msg::PolygonStamped & polygon, std::vector<Point> & data) const
{
    // Iterate over the vertices of the polygon
    for (const auto& point : polygon.polygon.points) {
        Point p;
        p.x = point.x;
        p.y = point.y;
        data.push_back(p);
    }

    // Add additional points sampled across the vertices of the polygon
    for (size_t i = 0; i < polygon.polygon.points.size(); i++) {
        size_t next_index = (i + 1) % polygon.polygon.points.size();

        // Sample 10 points between each pair of vertices
        for (size_t j = 0; j < 10; j++) {
            double t = static_cast<double>(j) / 10;
            Point p;
            p.x = polygon.polygon.points[i].x + t * (polygon.polygon.points[next_index].x - polygon.polygon.points[i].x);
            p.y = polygon.polygon.points[i].y + t * (polygon.polygon.points[next_index].y - polygon.polygon.points[i].y);
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

  // nav2_util::declare_parameter_if_not_declared(
  //   node, source_name_ + ".min_height", rclcpp::ParameterValue(0.05));
  // min_height_ = node->get_parameter(source_name_ + ".min_height").as_double();
  // nav2_util::declare_parameter_if_not_declared(
  //   node, source_name_ + ".max_height", rclcpp::ParameterValue(0.5));
  // max_height_ = node->get_parameter(source_name_ + ".max_height").as_double();
}

void PolygonSource::dataCallback(nav2_msgs::msg::PolygonsArray::ConstSharedPtr msg)
{
  data_ = msg;
}

}  // namespace nav2_collision_monitor
