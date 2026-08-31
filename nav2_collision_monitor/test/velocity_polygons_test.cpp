// Copyright (c) 2024 Dexory
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

#include <gtest/gtest.h>

#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "nav2_ros_common/lifecycle_node.hpp"
#include "geometry_msgs/msg/point32.hpp"
#include "geometry_msgs/msg/polygon_stamped.hpp"

#include "tf2_ros/buffer.hpp"
#include "tf2_ros/transform_listener.hpp"
#include "tf2_ros/transform_broadcaster.hpp"

#include "nav2_collision_monitor/types.hpp"
#include "nav2_collision_monitor/polygon.hpp"
#include "nav2_collision_monitor/velocity_polygon.hpp"

using namespace std::chrono_literals;

static constexpr double EPSILON = std::numeric_limits<float>::epsilon();

static const char BASE_FRAME_ID[]{"base_link"};
static const char POLYGON_PUB_TOPIC[]{"polygon_pub"};
static const char POLYGON_NAME[]{"TestVelocityPolygon"};
static const char SUB_POLYGON_FORWARD_NAME[]{"Forward"};
static const char SUB_POLYGON_BACKWARD_NAME[]{"Backward"};
static const char SUB_POLYGON_LEFT_NAME[]{"Left"};
static const char SUB_POLYGON_RIGHT_NAME[]{"Right"};
static const std::vector<double> FORWARD_POLYGON{
  0.5, 0.5, 0.5, -0.5, 0.0, -0.5, 0.0, 0.5};
static const std::vector<double> BACKWARD_POLYGON{
  0.0, 0.5, 0.0, -0.5, -0.5, -0.5, -0.5, 0.5};
static const std::vector<double> LEFT_POLYGON{
  0.5, 0.5, 0.5, 0.0, 0.0, 0.0, 0.0, -0.5};
static const std::vector<double> RIGHT_POLYGON{
  0.5, 0.0, 0.5, -0.5, -0.5, -0.5, 0.0, 0.0};
static const char FORWARD_POLYGON_STR[]{
  "[[0.5, 0.5], [0.5, -0.5], [0.0, -0.5], [0.0, 0.5]]"};
static const char BACKWARD_POLYGON_STR[]{
  "[[0.0, 0.5], [0.0, -0.5], [-0.5, -0.5], [-0.5, 0.5]]"};
static const char LEFT_POLYGON_STR[]{
  "[[0.5, 0.5], [0.5, 0.0], [0.0, 0.0], [0.0, -0.5]]"};
static const char RIGHT_POLYGON_STR[]{
  "[[0.5, 0.0], [0.5, -0.5], [-0.5, -0.5], [0.0, 0.0]]"};

static const bool IS_HOLONOMIC{true};
static const bool IS_NOT_HOLONOMIC{false};
static const int MIN_POINTS{2};
static const double SLOWDOWN_RATIO{0.25};
static const double LINEAR_LIMIT{0.3};
static const double ANGULAR_LIMIT{0.2};
static const double TIME_BEFORE_COLLISION{2.0};

static const tf2::Duration TRANSFORM_TOLERANCE{tf2::durationFromSec(0.1)};

class TestNode : public nav2::LifecycleNode
{
public:
  TestNode()
  : nav2::LifecycleNode("test_node"), polygon_received_(nullptr)
  {
  }

  ~TestNode() {}

  nav2::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & /*previous_state*/) override
  {
    polygon_sub_ = this->create_subscription<geometry_msgs::msg::PolygonStamped>(
      POLYGON_PUB_TOPIC,
      std::bind(&TestNode::polygonCallback, this, std::placeholders::_1));
    return nav2::CallbackReturn::SUCCESS;
  }

  void polygonCallback(geometry_msgs::msg::PolygonStamped::ConstSharedPtr msg)
  {
    polygon_received_ = msg;
  }

private:
  nav2::Subscription<geometry_msgs::msg::PolygonStamped>::SharedPtr polygon_sub_;
  geometry_msgs::msg::PolygonStamped::ConstSharedPtr polygon_received_;
};  // TestNode

class VelocityPolygonWrapper : public nav2_collision_monitor::VelocityPolygon
{
public:
  VelocityPolygonWrapper(
    const nav2::LifecycleNode::WeakPtr & node,
    const std::string & polygon_name,
    const std::shared_ptr<tf2_ros::Buffer> tf_buffer,
    const std::string & base_frame_id,
    const tf2::Duration & transform_tolerance)
  : nav2_collision_monitor::VelocityPolygon(
      node, polygon_name, tf_buffer, base_frame_id, transform_tolerance)
  {
  }

  double isHolonomic() const
  {
    return holonomic_;
  }

  double isVisualize() const
  {
    return visualize_;
  }

  std::vector<SubPolygonParameter> getSubPolygons()
  {
    return sub_polygons_;
  }

  double getCurrentSteeringAngle() const
  {
    return current_steering_angle_;
  }

  double callComputeSteeringAngle(const nav2_collision_monitor::Velocity & vel) const
  {
    return computeSteeringAngle(vel);
  }

  double callBaselinkToSteeringSpeed(double linear_vel, double angular_vel) const
  {
    return baselinkToSteeringSpeed(linear_vel, angular_vel);
  }

  double callSteeringToBaselinkSpeed(double steering_speed, double steering_angle) const
  {
    return steeringToBaselinkSpeed(steering_speed, steering_angle);
  }

  double callSteeringAngleToTw(double baselink_speed, double steering_angle) const
  {
    return steeringAngleToTw(baselink_speed, steering_angle);
  }

  const SubPolygonParameter * callFindField(
    double steering_wheel_speed, double steering_angle) const
  {
    return findField(steering_wheel_speed, steering_angle);
  }

  std::vector<const SubPolygonParameter *> callFindFieldsForAngle(
    double steering_angle, bool forward) const
  {
    return findFieldsForAngle(steering_angle, forward);
  }

  static bool callIsPointInsidePoly(
    const nav2_collision_monitor::Point & point,
    const std::vector<nav2_collision_monitor::Point> & vertices)
  {
    return isPointInsidePoly(point, vertices);
  }

  int callGetPointsInsideSubPolygon(
    const SubPolygonParameter & sub_polygon,
    const std::unordered_map<std::string,
    std::vector<nav2_collision_monitor::Point>> & collision_points_map) const
  {
    return getPointsInsideSubPolygon(sub_polygon, collision_points_map);
  }

  bool callClampToMaxField(
    const nav2_collision_monitor::Velocity & odom_vel,
    nav2_collision_monitor::Action & robot_action)
  {
    return clampToMaxField(odom_vel, robot_action);
  }
};  // VelocityPolygonWrapper

class Tester : public ::testing::Test
{
public:
  Tester();
  ~Tester();

protected:
  // Working with parameters
  void setCommonParameters(const std::string & polygon_name, const std::string & action_type);
  void addSlowdownParameters(const std::string & polygon_name);
  void addLimitParameters(const std::string & polygon_name);
  void addApproachParameters(const std::string & polygon_name);

  void setVelocityPolygonParameters(const bool is_holonomic);
  void addPolygonVelocitySubPolygon(
    const std::string & sub_polygon_name,
    const double linear_min, const double linear_max,
    const double theta_min, const double theta_max,
    const double direction_end_angle, const double direction_start_angle,
    const std::string & polygon_points, const bool is_holonomic,
    const std::vector<std::string> & modes = {});

  void addSteeringAngleSubPolygon(
    const std::string & sub_polygon_name,
    const double linear_min, const double linear_max,
    const double steering_angle_min, const double steering_angle_max,
    const std::string & polygon_points,
    const std::vector<std::string> & modes = {});
  void setSteeringVelocityPolygonParameters(
    const double wheelbase, const double low_speed_threshold,
    const std::vector<std::string> & sub_polygon_names);
  void addSteeringStaircase();

  // Creating routines
  void createVelocityPolygon(const std::string & action_type, const bool is_holonomic);
  void createSteeringVelocityPolygon(const std::string & action_type);
  void createVelocityPolygonWithModes(const std::string & action_type);

  // Wait until polygon will be received
  bool waitPolygon(
    const std::chrono::nanoseconds & timeout,
    std::vector<nav2_collision_monitor::Point> & poly);

  std::shared_ptr<TestNode> test_node_;
  rclcpp::executors::SingleThreadedExecutor::SharedPtr executor_;

  std::shared_ptr<VelocityPolygonWrapper> velocity_polygon_;

  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
};  // Tester

Tester::Tester()
{
  test_node_ = std::make_shared<TestNode>();
  executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
  executor_->add_node(test_node_->get_node_base_interface());
  test_node_->configure();
  test_node_->activate();

  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(test_node_->get_clock());
  tf_buffer_->setUsingDedicatedThread(true);  // One-thread broadcasting-listening model
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
}

Tester::~Tester()
{
  velocity_polygon_.reset();

  test_node_.reset();

  tf_listener_.reset();
  tf_buffer_.reset();
}

void Tester::setCommonParameters(const std::string & polygon_name, const std::string & action_type)
{
  test_node_->declare_parameter(
    polygon_name + ".action_type", rclcpp::ParameterValue(action_type));

  test_node_->declare_parameter(
    polygon_name + ".min_points", rclcpp::ParameterValue(MIN_POINTS));

  test_node_->declare_parameter(
    polygon_name + ".visualize", rclcpp::ParameterValue(true));

  test_node_->declare_parameter(
    polygon_name + ".polygon_pub_topic", rclcpp::ParameterValue(POLYGON_PUB_TOPIC));

  std::vector<std::string> default_observation_sources = {"source"};
  test_node_->declare_parameter(
    "observation_sources", rclcpp::ParameterValue(default_observation_sources));
}

void Tester::addSlowdownParameters(const std::string & polygon_name)
{
  test_node_->set_parameter(
    rclcpp::Parameter(std::string(POLYGON_NAME) + "." + polygon_name + ".slowdown_ratio", SLOWDOWN_RATIO));
}

void Tester::addLimitParameters(const std::string & polygon_name)
{
  test_node_->set_parameter(
    rclcpp::Parameter(std::string(POLYGON_NAME) + "." + polygon_name + ".linear_limit", LINEAR_LIMIT));

  test_node_->set_parameter(
    rclcpp::Parameter(std::string(POLYGON_NAME) + "." + polygon_name + ".angular_limit", ANGULAR_LIMIT));
}

void Tester::addApproachParameters(const std::string & polygon_name)
{
  test_node_->set_parameter(
    rclcpp::Parameter(
      std::string(
        POLYGON_NAME) + "." + polygon_name + ".time_before_collision", TIME_BEFORE_COLLISION));

}

void Tester::setVelocityPolygonParameters(const bool is_holonomic)
{
  test_node_->declare_parameter(
    std::string(POLYGON_NAME) + ".holonomic", rclcpp::ParameterValue(is_holonomic));

  std::vector<std::string> velocity_polygons =
  {SUB_POLYGON_FORWARD_NAME, SUB_POLYGON_BACKWARD_NAME};

  if (is_holonomic) {
    // Direction angle range for holonomic type
    //
    //                    ^OY
    //                    |
    //                    |
    //        0.75pi    (left)    0.25pi
    //             ---------------  <- robot footprint
    //             | \    |    / |
    //  (backward) |   \  |  /   | (forward)
    // --------pi--|------o------|---------->OX
    //             |   /  | \    |
    //             | /    |   \  |
    //             --------------
    //       -0.75pi   (right)    -0.25pi
    //                    |
    addPolygonVelocitySubPolygon(
      SUB_POLYGON_FORWARD_NAME, 0.0, 0.5, -1.0, 1.0, -M_PI_4, M_PI_4, FORWARD_POLYGON_STR,
      is_holonomic);
    addPolygonVelocitySubPolygon(
      SUB_POLYGON_BACKWARD_NAME, 0.0, 0.5, -1.0, 1.0, 0.75 * M_PI, -0.75 * M_PI,
      BACKWARD_POLYGON_STR,
      is_holonomic);
    addPolygonVelocitySubPolygon(
      SUB_POLYGON_LEFT_NAME, 0.0, 0.5, -1.0, 1.0, M_PI_4, 0.75 * M_PI, LEFT_POLYGON_STR,
      is_holonomic);
    addPolygonVelocitySubPolygon(
      SUB_POLYGON_RIGHT_NAME, 0.0, 0.5, -1.0, 1.0, -0.75 * M_PI, -M_PI_4,
      RIGHT_POLYGON_STR, is_holonomic);

    velocity_polygons = {SUB_POLYGON_FORWARD_NAME, SUB_POLYGON_BACKWARD_NAME, SUB_POLYGON_LEFT_NAME,
      SUB_POLYGON_RIGHT_NAME};
  } else {
    // draw forward and backward polygon
    addPolygonVelocitySubPolygon(
      SUB_POLYGON_FORWARD_NAME, 0.0, 0.5, -1.0, 1.0, 0.0, 0.0, FORWARD_POLYGON_STR,
      is_holonomic);
    addPolygonVelocitySubPolygon(
      SUB_POLYGON_BACKWARD_NAME, -0.5, 0.0, -1.0, 1.0, 0.0, 0.0, BACKWARD_POLYGON_STR,
      is_holonomic);
    velocity_polygons = {SUB_POLYGON_FORWARD_NAME, SUB_POLYGON_BACKWARD_NAME};
  }

  test_node_->declare_parameter(
    std::string(POLYGON_NAME) + ".velocity_polygons", rclcpp::ParameterValue(velocity_polygons));
}

void Tester::addPolygonVelocitySubPolygon(
  const std::string & sub_polygon_name,
  const double linear_min, const double linear_max,
  const double theta_min, const double theta_max,
  const double direction_start_angle, const double direction_end_angle,
  const std::string & polygon_points, const bool is_holonomic,
  const std::vector<std::string> & modes)
{
  test_node_->declare_parameter(
    std::string(POLYGON_NAME) + "." + sub_polygon_name + ".points",
    rclcpp::ParameterValue(polygon_points));

  test_node_->declare_parameter(
    std::string(POLYGON_NAME) + "." + sub_polygon_name + ".linear_min",
    rclcpp::ParameterValue(linear_min));

  test_node_->declare_parameter(
    std::string(POLYGON_NAME) + "." + sub_polygon_name + ".linear_max",
    rclcpp::ParameterValue(linear_max));

  test_node_->declare_parameter(
    std::string(POLYGON_NAME) + "." + sub_polygon_name + ".theta_min",
    rclcpp::ParameterValue(theta_min));

  test_node_->declare_parameter(
    std::string(POLYGON_NAME) + "." + sub_polygon_name + ".theta_max",
    rclcpp::ParameterValue(theta_max));

  if (is_holonomic) {
    test_node_->declare_parameter(
      std::string(
        POLYGON_NAME) +
      "." + sub_polygon_name + ".direction_end_angle",
      rclcpp::ParameterValue(direction_end_angle));

    test_node_->declare_parameter(
      std::string(
        POLYGON_NAME) +
      "." + sub_polygon_name + ".direction_start_angle",
      rclcpp::ParameterValue(direction_start_angle));
  }

  if (!modes.empty()) {
    test_node_->declare_parameter(
      std::string(POLYGON_NAME) + "." + sub_polygon_name + ".modes",
      rclcpp::ParameterValue(modes));
  }
}

void Tester::addSteeringAngleSubPolygon(
  const std::string & sub_polygon_name,
  const double linear_min, const double linear_max,
  const double steering_angle_min, const double steering_angle_max,
  const std::string & polygon_points,
  const std::vector<std::string> & modes)
{
  const std::string prefix = std::string(POLYGON_NAME) + "." + sub_polygon_name;

  test_node_->declare_parameter(
    prefix + ".points", rclcpp::ParameterValue(polygon_points));
  test_node_->set_parameter(
    rclcpp::Parameter(prefix + ".points", polygon_points));

  test_node_->declare_parameter(
    prefix + ".linear_min", rclcpp::ParameterValue(linear_min));
  test_node_->set_parameter(
    rclcpp::Parameter(prefix + ".linear_min", linear_min));

  test_node_->declare_parameter(
    prefix + ".linear_max", rclcpp::ParameterValue(linear_max));
  test_node_->set_parameter(
    rclcpp::Parameter(prefix + ".linear_max", linear_max));

  test_node_->declare_parameter(
    prefix + ".steering_angle_min", rclcpp::ParameterValue(steering_angle_min));
  test_node_->set_parameter(
    rclcpp::Parameter(prefix + ".steering_angle_min", steering_angle_min));

  test_node_->declare_parameter(
    prefix + ".steering_angle_max", rclcpp::ParameterValue(steering_angle_max));
  test_node_->set_parameter(
    rclcpp::Parameter(prefix + ".steering_angle_max", steering_angle_max));

  if (!modes.empty()) {
    test_node_->declare_parameter(prefix + ".modes", rclcpp::ParameterValue(modes));
    test_node_->set_parameter(rclcpp::Parameter(prefix + ".modes", modes));
  }
}

void Tester::setSteeringVelocityPolygonParameters(
  const double wheelbase, const double low_speed_threshold,
  const std::vector<std::string> & sub_polygon_names)
{
  test_node_->declare_parameter(
    std::string(POLYGON_NAME) + ".holonomic", rclcpp::ParameterValue(false));
  test_node_->set_parameter(
    rclcpp::Parameter(std::string(POLYGON_NAME) + ".holonomic", false));

  test_node_->declare_parameter(
    std::string(POLYGON_NAME) + ".wheelbase", rclcpp::ParameterValue(wheelbase));
  test_node_->set_parameter(
    rclcpp::Parameter(std::string(POLYGON_NAME) + ".wheelbase", wheelbase));

  test_node_->declare_parameter(
    std::string(POLYGON_NAME) + ".low_speed_threshold",
    rclcpp::ParameterValue(low_speed_threshold));
  test_node_->set_parameter(
    rclcpp::Parameter(
      std::string(POLYGON_NAME) + ".low_speed_threshold", low_speed_threshold));

  test_node_->declare_parameter(
    std::string(POLYGON_NAME) + ".velocity_polygons",
    rclcpp::ParameterValue(sub_polygon_names));
  test_node_->set_parameter(
    rclcpp::Parameter(std::string(POLYGON_NAME) + ".velocity_polygons", sub_polygon_names));
}

void Tester::createSteeringVelocityPolygon(const std::string & action_type)
{
  setCommonParameters(POLYGON_NAME, action_type);

  velocity_polygon_ = std::make_shared<VelocityPolygonWrapper>(
    test_node_, POLYGON_NAME,
    tf_buffer_, BASE_FRAME_ID, TRANSFORM_TOLERANCE);
  ASSERT_TRUE(velocity_polygon_->configure());
  velocity_polygon_->activate();
}

void Tester::createVelocityPolygon(const std::string & action_type, const bool is_holonomic)
{
  setCommonParameters(POLYGON_NAME, action_type);
  setVelocityPolygonParameters(is_holonomic);

  velocity_polygon_ = std::make_shared<VelocityPolygonWrapper>(
    test_node_->weak_from_this(), POLYGON_NAME,
    tf_buffer_, BASE_FRAME_ID, TRANSFORM_TOLERANCE);
  ASSERT_TRUE(velocity_polygon_->configure());
  velocity_polygon_->activate();
}

void Tester::createVelocityPolygonWithModes(const std::string & action_type)
{
  setCommonParameters(POLYGON_NAME, action_type);

  test_node_->declare_parameter(
    std::string(POLYGON_NAME) + ".holonomic", rclcpp::ParameterValue(false));

  static const char FWD_DEFAULT[]{"ForwardDefault"};
  static const char FWD_FORKDOWN[]{"ForwardForkDown"};
  static const char BWD[]{"Backward"};

  std::vector<std::string> velocity_polygons = {FWD_DEFAULT, FWD_FORKDOWN, BWD};
  test_node_->declare_parameter(
    std::string(POLYGON_NAME) + ".velocity_polygons", rclcpp::ParameterValue(velocity_polygons));

  addPolygonVelocitySubPolygon(
    FWD_DEFAULT, 0.0, 1.0, -1.0, 1.0, 0.0, 0.0, FORWARD_POLYGON_STR,
    false, {"default"});
  addPolygonVelocitySubPolygon(
    FWD_FORKDOWN, 0.0, 0.5, -1.0, 1.0, 0.0, 0.0, FORWARD_POLYGON_STR,
    false, {"fork_down"});
  addPolygonVelocitySubPolygon(
    BWD, -1.0, 0.0, -1.0, 1.0, 0.0, 0.0, BACKWARD_POLYGON_STR,
    false, {"default", "fork_down"});

  velocity_polygon_ = std::make_shared<VelocityPolygonWrapper>(
    test_node_->weak_from_this(), POLYGON_NAME,
    tf_buffer_, BASE_FRAME_ID, TRANSFORM_TOLERANCE);
  ASSERT_TRUE(velocity_polygon_->configure());
  velocity_polygon_->activate();
}

bool Tester::waitPolygon(
  const std::chrono::nanoseconds & timeout,
  std::vector<nav2_collision_monitor::Point> & poly)
{
  rclcpp::Time start_time = test_node_->now();
  while (rclcpp::ok() && test_node_->now() - start_time <= rclcpp::Duration(timeout)) {
    velocity_polygon_->getPolygon(poly);
    if (poly.size() > 0) {
      return true;
    }
    executor_->spin_some();
    std::this_thread::sleep_for(10ms);
  }
  return false;
}

TEST_F(Tester, testVelocityPolygonGetStopParameters)
{
  createVelocityPolygon("stop", IS_NOT_HOLONOMIC);

  // Check that common parameters set correctly
  EXPECT_EQ(velocity_polygon_->getName(), POLYGON_NAME);
  EXPECT_EQ(velocity_polygon_->getActionType(), nav2_collision_monitor::STOP);
  EXPECT_EQ(velocity_polygon_->getMinPoints(), MIN_POINTS);
  EXPECT_EQ(velocity_polygon_->isVisualize(), true);
}

TEST_F(Tester, testVelocityPolygonGetSlowdownParameters)
{
  createVelocityPolygon("slowdown", IS_NOT_HOLONOMIC);

  // Check that common parameters set correctly
  EXPECT_EQ(velocity_polygon_->getName(), POLYGON_NAME);
  EXPECT_EQ(velocity_polygon_->getActionType(), nav2_collision_monitor::SLOWDOWN);
  EXPECT_EQ(velocity_polygon_->getMinPoints(), MIN_POINTS);
  EXPECT_EQ(velocity_polygon_->isVisualize(), true);
}

TEST_F(Tester, testVelocityPolygonParameters)
{
  createVelocityPolygon("stop", IS_NOT_HOLONOMIC);

  // Check velocity polygon parameters
  EXPECT_EQ(velocity_polygon_->isHolonomic(), IS_NOT_HOLONOMIC);
  ASSERT_EQ(velocity_polygon_->getSubPolygons().size(), 2u);
}

TEST_F(Tester, testHolonomicVelocityPolygonParameters)
{
  createVelocityPolygon("stop", IS_HOLONOMIC);

  // Check velocity polygon parameters
  EXPECT_EQ(velocity_polygon_->isHolonomic(), IS_HOLONOMIC);
  ASSERT_EQ(velocity_polygon_->getSubPolygons().size(), 4u);
}

TEST_F(Tester, testVelocityPolygonOutOfRangeVelocity)
{
  createVelocityPolygon("stop", IS_NOT_HOLONOMIC);

  // Check velocity polygon parameters
  EXPECT_EQ(velocity_polygon_->isHolonomic(), IS_NOT_HOLONOMIC);
  ASSERT_EQ(velocity_polygon_->getSubPolygons().size(), 2u);

  // Check that polygon is empty before the first cmd_vel received
  std::vector<nav2_collision_monitor::Point> poly;
  velocity_polygon_->getPolygon(poly);
  ASSERT_EQ(poly.size(), 0u);


  // Publish out of range cmd_vel(linear) and check that polygon is still empty
  nav2_collision_monitor::Velocity vel{0.6, 0.0, 0.0};
  velocity_polygon_->updatePolygon(vel);
  ASSERT_FALSE(waitPolygon(500ms, poly));
  ASSERT_EQ(poly.size(), 0u);

  // Publish out of range cmd_vel(rotation) and check that polygon is still empty
  vel = {0.3, 0.0, 1.5};
  velocity_polygon_->updatePolygon(vel);
  ASSERT_FALSE(waitPolygon(500ms, poly));
  ASSERT_EQ(poly.size(), 0u);

  // Publish a valid cmd_vel and check that polygon is correct
  vel = {0.3, 0.0, 0.0};  // 0.3 m/s forward movement
  velocity_polygon_->updatePolygon(vel);
  ASSERT_TRUE(waitPolygon(500ms, poly));
  ASSERT_EQ(poly.size(), 4u);
}

TEST_F(Tester, testVelocityPolygonVelocitySwitching)
{
  createVelocityPolygon("stop", IS_NOT_HOLONOMIC);

  // Check velocity polygon parameters
  EXPECT_EQ(velocity_polygon_->isHolonomic(), IS_NOT_HOLONOMIC);
  ASSERT_EQ(velocity_polygon_->getSubPolygons().size(), 2u);

  // Check that polygon is empty before the first cmd_vel received
  std::vector<nav2_collision_monitor::Point> poly;
  velocity_polygon_->getPolygon(poly);
  ASSERT_EQ(poly.size(), 0u);

  // Publish cmd_vel (forward) and check that polygon is correct
  nav2_collision_monitor::Velocity vel{0.3, 0.0, 0.0};
  velocity_polygon_->updatePolygon(vel);
  ASSERT_TRUE(waitPolygon(500ms, poly));
  ASSERT_EQ(poly.size(), 4u);
  EXPECT_NEAR(poly[0].x, FORWARD_POLYGON[0], EPSILON);
  EXPECT_NEAR(poly[0].y, FORWARD_POLYGON[1], EPSILON);
  EXPECT_NEAR(poly[1].x, FORWARD_POLYGON[2], EPSILON);
  EXPECT_NEAR(poly[1].y, FORWARD_POLYGON[3], EPSILON);
  EXPECT_NEAR(poly[2].x, FORWARD_POLYGON[4], EPSILON);
  EXPECT_NEAR(poly[2].y, FORWARD_POLYGON[5], EPSILON);
  EXPECT_NEAR(poly[3].x, FORWARD_POLYGON[6], EPSILON);
  EXPECT_NEAR(poly[3].y, FORWARD_POLYGON[7], EPSILON);

  // Publish cmd_vel (backward) and check that polygon is correct
  vel = {-0.3, 0.0, 0.0};  // 0.3 m/s backward movement
  velocity_polygon_->updatePolygon(vel);
  ASSERT_TRUE(waitPolygon(500ms, poly));
  ASSERT_EQ(poly.size(), 4u);
  EXPECT_NEAR(poly[0].x, BACKWARD_POLYGON[0], EPSILON);
  EXPECT_NEAR(poly[0].y, BACKWARD_POLYGON[1], EPSILON);
  EXPECT_NEAR(poly[1].x, BACKWARD_POLYGON[2], EPSILON);
  EXPECT_NEAR(poly[1].y, BACKWARD_POLYGON[3], EPSILON);
  EXPECT_NEAR(poly[2].x, BACKWARD_POLYGON[4], EPSILON);
  EXPECT_NEAR(poly[2].y, BACKWARD_POLYGON[5], EPSILON);
  EXPECT_NEAR(poly[3].x, BACKWARD_POLYGON[6], EPSILON);
  EXPECT_NEAR(poly[3].y, BACKWARD_POLYGON[7], EPSILON);
}

TEST_F(Tester, testVelocityPolygonHolonomicVelocitySwitching)
{
  createVelocityPolygon("stop", IS_HOLONOMIC);

  // Check velocity polygon parameters
  EXPECT_EQ(velocity_polygon_->isHolonomic(), IS_HOLONOMIC);
  ASSERT_EQ(velocity_polygon_->getSubPolygons().size(), 4u);

  // Check that polygon is empty before the first cmd_vel received
  std::vector<nav2_collision_monitor::Point> poly;
  velocity_polygon_->getPolygon(poly);
  ASSERT_EQ(poly.size(), 0u);

  // Publish cmd_vel (forward) and check that polygon is correct
  nav2_collision_monitor::Velocity vel{0.3, 0.0, 0.0};
  velocity_polygon_->updatePolygon(vel);
  ASSERT_TRUE(waitPolygon(500ms, poly));
  ASSERT_EQ(poly.size(), 4u);
  EXPECT_NEAR(poly[0].x, FORWARD_POLYGON[0], EPSILON);
  EXPECT_NEAR(poly[0].y, FORWARD_POLYGON[1], EPSILON);
  EXPECT_NEAR(poly[1].x, FORWARD_POLYGON[2], EPSILON);
  EXPECT_NEAR(poly[1].y, FORWARD_POLYGON[3], EPSILON);
  EXPECT_NEAR(poly[2].x, FORWARD_POLYGON[4], EPSILON);
  EXPECT_NEAR(poly[2].y, FORWARD_POLYGON[5], EPSILON);
  EXPECT_NEAR(poly[3].x, FORWARD_POLYGON[6], EPSILON);
  EXPECT_NEAR(poly[3].y, FORWARD_POLYGON[7], EPSILON);

  // Publish cmd_vel (backward) and check that polygon is correct
  vel = {-0.3, 0.0, 0.0};  // 0.3 m/s backward movement
  velocity_polygon_->updatePolygon(vel);
  ASSERT_TRUE(waitPolygon(500ms, poly));
  ASSERT_EQ(poly.size(), 4u);
  EXPECT_NEAR(poly[0].x, BACKWARD_POLYGON[0], EPSILON);
  EXPECT_NEAR(poly[0].y, BACKWARD_POLYGON[1], EPSILON);
  EXPECT_NEAR(poly[1].x, BACKWARD_POLYGON[2], EPSILON);
  EXPECT_NEAR(poly[1].y, BACKWARD_POLYGON[3], EPSILON);
  EXPECT_NEAR(poly[2].x, BACKWARD_POLYGON[4], EPSILON);
  EXPECT_NEAR(poly[2].y, BACKWARD_POLYGON[5], EPSILON);
  EXPECT_NEAR(poly[3].x, BACKWARD_POLYGON[6], EPSILON);
  EXPECT_NEAR(poly[3].y, BACKWARD_POLYGON[7], EPSILON);

  // Publish cmd_vel (left) and check that polygon is correct
  vel = {0.0, 0.3, 0.0};  // 0.3 m/s left movement
  velocity_polygon_->updatePolygon(vel);
  ASSERT_TRUE(waitPolygon(500ms, poly));
  ASSERT_EQ(poly.size(), 4u);
  EXPECT_NEAR(poly[0].x, LEFT_POLYGON[0], EPSILON);
  EXPECT_NEAR(poly[0].y, LEFT_POLYGON[1], EPSILON);
  EXPECT_NEAR(poly[1].x, LEFT_POLYGON[2], EPSILON);
  EXPECT_NEAR(poly[1].y, LEFT_POLYGON[3], EPSILON);
  EXPECT_NEAR(poly[2].x, LEFT_POLYGON[4], EPSILON);
  EXPECT_NEAR(poly[2].y, LEFT_POLYGON[5], EPSILON);
  EXPECT_NEAR(poly[3].x, LEFT_POLYGON[6], EPSILON);
  EXPECT_NEAR(poly[3].y, LEFT_POLYGON[7], EPSILON);

  // Publish cmd_vel (right) and check that polygon is correct
  vel = {0.0, -0.3, 0.0};  // 0.3 m/s right movement
  velocity_polygon_->updatePolygon(vel);
  ASSERT_TRUE(waitPolygon(500ms, poly));
  ASSERT_EQ(poly.size(), 4u);
  EXPECT_NEAR(poly[0].x, RIGHT_POLYGON[0], EPSILON);
  EXPECT_NEAR(poly[0].y, RIGHT_POLYGON[1], EPSILON);
  EXPECT_NEAR(poly[1].x, RIGHT_POLYGON[2], EPSILON);
  EXPECT_NEAR(poly[1].y, RIGHT_POLYGON[3], EPSILON);
  EXPECT_NEAR(poly[2].x, RIGHT_POLYGON[4], EPSILON);
  EXPECT_NEAR(poly[2].y, RIGHT_POLYGON[5], EPSILON);
  EXPECT_NEAR(poly[3].x, RIGHT_POLYGON[6], EPSILON);
  EXPECT_NEAR(poly[3].y, RIGHT_POLYGON[7], EPSILON);
}


// ==================== Steering wheel speed conversion tests ====================

// Polygon for steering tests: a simple square around the robot
static const char STEERING_POLYGON_STR[]{
  "[[1.0, 0.5], [1.0, -0.5], [-0.5, -0.5], [-0.5, 0.5]]"};
// Smaller polygon for the slower field
static const char STEERING_POLYGON_SLOW_STR[]{
  "[[0.5, 0.3], [0.5, -0.3], [-0.3, -0.3], [-0.3, 0.3]]"};
// Larger polygon for the faster field
static const char STEERING_POLYGON_FAST_STR[]{
  "[[1.5, 0.8], [1.5, -0.8], [-0.8, -0.8], [-0.8, 0.8]]"};

static const double WHEELBASE{1.0};
static const double LOW_SPEED_THRESHOLD{0.1};

TEST_F(Tester, testBaselinkToSteeringSpeedConversion)
{
  // Setup: create a simple steering velocity polygon
  setSteeringVelocityPolygonParameters(WHEELBASE, LOW_SPEED_THRESHOLD, {"slow", "fast"});
  addSteeringAngleSubPolygon("slow", 0.0, 0.5, -0.5, 0.5, STEERING_POLYGON_SLOW_STR);
  addSteeringAngleSubPolygon("fast", 0.5, 1.0, -0.5, 0.5, STEERING_POLYGON_FAST_STR);
  createSteeringVelocityPolygon("limit");

  // At zero angular vel, steering speed == baselink speed
  EXPECT_NEAR(velocity_polygon_->callBaselinkToSteeringSpeed(1.0, 0.0), 1.0, 1e-6);
  EXPECT_NEAR(velocity_polygon_->callBaselinkToSteeringSpeed(-0.5, 0.0), -0.5, 1e-6);

  // At 60 deg steering: tw = vx * tan(60°) / wheelbase = sqrt(3), expected = hypot(1, sqrt(3)) = 2
  EXPECT_NEAR(velocity_polygon_->callBaselinkToSteeringSpeed(1.0, std::sqrt(3.0)), 2.0, 1e-6);

  // At 45 deg steering: tw = vx * tan(45°) / wheelbase = 1.0, expected = hypot(1, 1) = sqrt(2)
  double expected = std::sqrt(2.0);
  EXPECT_NEAR(velocity_polygon_->callBaselinkToSteeringSpeed(1.0, 1.0), expected, 1e-6);

  // At 90 deg steering (vx=0, tw!=0): continuous result, not infinity
  double result = velocity_polygon_->callBaselinkToSteeringSpeed(0.0, 1.0);
  EXPECT_NEAR(result, WHEELBASE * 1.0, 1e-6);

  // Negative speed: sign is preserved
  EXPECT_NEAR(velocity_polygon_->callBaselinkToSteeringSpeed(-1.0, 1.0), -expected, 1e-6);
}

TEST_F(Tester, testSteeringToBaselinkSpeedConversion)
{
  setSteeringVelocityPolygonParameters(WHEELBASE, LOW_SPEED_THRESHOLD, {"slow"});
  addSteeringAngleSubPolygon("slow", 0.0, 0.5, -0.5, 0.5, STEERING_POLYGON_SLOW_STR);
  createSteeringVelocityPolygon("limit");

  // At zero steering angle, baselink speed == steering speed
  EXPECT_NEAR(velocity_polygon_->callSteeringToBaselinkSpeed(1.0, 0.0), 1.0, 1e-6);

  // At 60 degrees, cos(60°)=0.5, so baselink = steering * 0.5
  EXPECT_NEAR(velocity_polygon_->callSteeringToBaselinkSpeed(1.0, M_PI / 3), 0.5, 1e-6);

  // Roundtrip: baselink -> steering -> baselink should be identity
  double sa = 0.3;  // steering angle
  double baselink = 0.8;
  // tw = |vx| * tan(sa) / wheelbase (for vx > 0, wheelbase = 1.0)
  double tw = baselink * std::tan(sa) / WHEELBASE;
  double steering = velocity_polygon_->callBaselinkToSteeringSpeed(baselink, tw);
  double roundtrip = velocity_polygon_->callSteeringToBaselinkSpeed(steering, sa);
  EXPECT_NEAR(roundtrip, baselink, 1e-6);
}

TEST_F(Tester, testSteeringAngleToTw)
{
  setSteeringVelocityPolygonParameters(WHEELBASE, LOW_SPEED_THRESHOLD, {"slow"});
  addSteeringAngleSubPolygon("slow", 0.0, 0.5, -0.5, 0.5, STEERING_POLYGON_SLOW_STR);
  createSteeringVelocityPolygon("limit");

  // At zero steering angle, tw should be 0
  EXPECT_NEAR(velocity_polygon_->callSteeringAngleToTw(1.0, 0.0), 0.0, 1e-6);

  // Forward, positive steering angle → positive tw
  // tw = tan(angle) * |v| / wheelbase
  double sa = 0.3;
  double v = 1.0;
  double expected_tw = std::tan(sa) * std::abs(v) / WHEELBASE;
  EXPECT_NEAR(velocity_polygon_->callSteeringAngleToTw(v, sa), expected_tw, 1e-6);

  // Reverse, positive steering angle → negative tw (sign-corrected for reverse)
  double expected_tw_reverse = -(std::tan(sa) * std::abs(-v) / WHEELBASE);
  EXPECT_NEAR(velocity_polygon_->callSteeringAngleToTw(-v, sa), expected_tw_reverse, 1e-6);
}

// Verify that swToBaselink (steeringToBaselinkSpeed + steeringAngleToTw) is the
// exact inverse of tricycle_controller's twist_to_ackermann.
// twist_to_ackermann (with wheel_radius=1):
//   alpha = atan(theta_dot * wheelbase / Vx)
//   Ws    = Vx / (wheel_radius * cos(alpha))   [== sw_speed when wheel_radius=1]
// swToBaselink:
//   Vx        = steeringToBaselinkSpeed(sw, sa) = sw * cos(sa)
//   theta_dot = steeringAngleToTw(Vx, sa)       = tan(sa) * |Vx| / wheelbase
TEST_F(Tester, testSwToBaselinkInverseOfTwistToAckermann)
{
  setSteeringVelocityPolygonParameters(WHEELBASE, LOW_SPEED_THRESHOLD, {"slow"});
  addSteeringAngleSubPolygon("slow", 0.0, 1.0, -1.0, 1.0, STEERING_POLYGON_SLOW_STR);
  createSteeringVelocityPolygon("limit");

  // Inline twist_to_ackermann logic (from tricycle_controller) with wheel_radius=1.0
  const double wheel_radius = 1.0;
  auto twist_to_ackermann = [&](double Vx, double theta_dot)
    -> std::pair<double, double> {
      if (Vx == 0 && theta_dot != 0) {
        double alpha = theta_dot > 0 ? M_PI_2 : -M_PI_2;
        double Ws = std::abs(theta_dot) * WHEELBASE / wheel_radius;
        return {alpha, Ws};
      }
      double alpha = (theta_dot == 0 || Vx == 0) ?
        0.0 : std::atan(theta_dot * WHEELBASE / Vx);
      double Ws = Vx / (wheel_radius * std::cos(alpha));
      return {alpha, Ws};
    };

  // Test cases: (sw_speed, steering_angle) → swToBaselink → twist_to_ackermann → roundtrip
  struct TestCase { double sw; double sa; };
  std::vector<TestCase> cases = {
    {1.0, 0.0},        // straight forward
    {0.5, 0.3},        // moderate turn forward
    {0.8, -0.4},       // moderate turn other direction
    {-0.6, 0.2},       // reverse with steering
    {-1.0, 0.0},       // straight reverse
    {-0.5, -0.3},      // reverse other direction
    {0.3, 0.8},        // sharp turn forward
    {2.0, 0.1},        // high speed slight turn
  };

  for (const auto & tc : cases) {
    // Step 1: swToBaselink
    double vx = velocity_polygon_->callSteeringToBaselinkSpeed(tc.sw, tc.sa);
    double tw = velocity_polygon_->callSteeringAngleToTw(vx, tc.sa);

    // Step 2: twist_to_ackermann (inverse)
    auto [alpha, Ws] = twist_to_ackermann(vx, tw);

    EXPECT_NEAR(alpha, tc.sa, 1e-9)
      << "Steering angle roundtrip failed for sw=" << tc.sw << " sa=" << tc.sa
      << " (vx=" << vx << " tw=" << tw << ")";
    EXPECT_NEAR(Ws, tc.sw, 1e-9)
      << "Steering wheel speed roundtrip failed for sw=" << tc.sw << " sa=" << tc.sa
      << " (vx=" << vx << " tw=" << tw << ")";
  }
}

TEST_F(Tester, testIsInRangeWithSteeringWheelSpeed)
{
  // Setup: 2 speed fields with steering angle params
  // Slow: 0.0 to 0.5 steering wheel speed, steering angle -0.5 to 0.5
  // Fast: 0.5 to 1.0 steering wheel speed, steering angle -0.5 to 0.5
  setSteeringVelocityPolygonParameters(WHEELBASE, LOW_SPEED_THRESHOLD, {"slow", "fast"});
  addSteeringAngleSubPolygon("slow", 0.0, 0.5, -0.5, 0.5, STEERING_POLYGON_SLOW_STR);
  addSteeringAngleSubPolygon("fast", 0.5, 1.0, -0.5, 0.5, STEERING_POLYGON_FAST_STR);
  createSteeringVelocityPolygon("limit");

  // With zero steering angle: baselink speed == steering wheel speed
  // 0.3 m/s baselink → 0.3 steering → should be "slow" field
  nav2_collision_monitor::Velocity vel{0.3, 0.0, 0.0};
  velocity_polygon_->updatePolygon(vel);
  EXPECT_EQ(velocity_polygon_->getCurrentSubPolygonName(), "slow");

  // 0.7 m/s baselink → 0.7 steering → should be "fast" field
  vel = {0.7, 0.0, 0.0};
  velocity_polygon_->updatePolygon(vel);
  EXPECT_EQ(velocity_polygon_->getCurrentSubPolygonName(), "fast");

  // With non-zero steering angle, the baselink speed that maps to a given
  // steering wheel speed is lower. E.g., at 0.3 rad steering angle:
  // v_steering = v_baselink / cos(0.3) ≈ v_baselink / 0.9553
  // So 0.48 baselink → 0.48 / 0.9553 ≈ 0.502 steering → "fast" field
  // We need to set tw correctly: tw = tan(sa) * |v| / wheelbase
  double sa = 0.3;
  double v_base = 0.48;
  double tw = std::tan(sa) * std::abs(v_base) / WHEELBASE;
  vel = {v_base, 0.0, tw};
  velocity_polygon_->updatePolygon(vel);
  EXPECT_EQ(velocity_polygon_->getCurrentSubPolygonName(), "fast");

  // With same steering angle, 0.45 baselink → 0.45 / 0.9553 ≈ 0.471 steering → "slow" field
  v_base = 0.45;
  tw = std::tan(sa) * std::abs(v_base) / WHEELBASE;
  vel = {v_base, 0.0, tw};
  velocity_polygon_->updatePolygon(vel);
  EXPECT_EQ(velocity_polygon_->getCurrentSubPolygonName(), "slow");
}

TEST_F(Tester, testLinearLimitConversionInUpdatePolygon)
{
  // Setup: create a LIMIT velocity polygon with steering angle
  setSteeringVelocityPolygonParameters(WHEELBASE, LOW_SPEED_THRESHOLD, {"slow"});

  // Add a sub-polygon with linear_limit = 0.49 (steering wheel speed)
  const std::string prefix = std::string(POLYGON_NAME) + ".slow";
  test_node_->declare_parameter(prefix + ".points", rclcpp::ParameterValue(
      std::string(STEERING_POLYGON_SLOW_STR)));
  test_node_->set_parameter(rclcpp::Parameter(prefix + ".points",
    std::string(STEERING_POLYGON_SLOW_STR)));
  test_node_->declare_parameter(prefix + ".linear_min", rclcpp::ParameterValue(0.0));
  test_node_->set_parameter(rclcpp::Parameter(prefix + ".linear_min", 0.0));
  test_node_->declare_parameter(prefix + ".linear_max", rclcpp::ParameterValue(1.0));
  test_node_->set_parameter(rclcpp::Parameter(prefix + ".linear_max", 1.0));
  test_node_->declare_parameter(prefix + ".steering_angle_min", rclcpp::ParameterValue(-0.5));
  test_node_->set_parameter(rclcpp::Parameter(prefix + ".steering_angle_min", -0.5));
  test_node_->declare_parameter(prefix + ".steering_angle_max", rclcpp::ParameterValue(0.5));
  test_node_->set_parameter(rclcpp::Parameter(prefix + ".steering_angle_max", 0.5));
  test_node_->declare_parameter(prefix + ".linear_limit", rclcpp::ParameterValue(0.49));
  test_node_->set_parameter(rclcpp::Parameter(prefix + ".linear_limit", 0.49));
  test_node_->declare_parameter(prefix + ".angular_limit", rclcpp::ParameterValue(0.5));
  test_node_->set_parameter(rclcpp::Parameter(prefix + ".angular_limit", 0.5));

  createSteeringVelocityPolygon("limit");

  // At zero steering angle, linear_limit should be 0.49 * cos(0) = 0.49
  nav2_collision_monitor::Velocity vel{0.3, 0.0, 0.0};
  velocity_polygon_->updatePolygon(vel);
  EXPECT_NEAR(velocity_polygon_->getLinearLimit(), 0.49, 1e-6);

  // At steering angle 0.3 rad, linear_limit should be 0.49 * cos(0.3) ≈ 0.4681
  double sa = 0.3;
  double v_base = 0.3;
  double tw = std::tan(sa) * std::abs(v_base) / WHEELBASE;
  vel = {v_base, 0.0, tw};
  velocity_polygon_->updatePolygon(vel);
  double expected_limit = 0.49 * std::cos(sa);
  EXPECT_NEAR(velocity_polygon_->getLinearLimit(), expected_limit, 1e-3);
}

TEST_F(Tester, testIsPointInsidePoly)
{
  // Simple square polygon: (-1,-1), (1,-1), (1,1), (-1,1)
  std::vector<nav2_collision_monitor::Point> square = {
    {-1.0, -1.0}, {1.0, -1.0}, {1.0, 1.0}, {-1.0, 1.0}
  };

  // Point inside
  EXPECT_TRUE(VelocityPolygonWrapper::callIsPointInsidePoly({0.0, 0.0}, square));
  EXPECT_TRUE(VelocityPolygonWrapper::callIsPointInsidePoly({0.5, 0.5}, square));

  // Point outside
  EXPECT_FALSE(VelocityPolygonWrapper::callIsPointInsidePoly({2.0, 0.0}, square));
  EXPECT_FALSE(VelocityPolygonWrapper::callIsPointInsidePoly({0.0, 2.0}, square));
  EXPECT_FALSE(VelocityPolygonWrapper::callIsPointInsidePoly({-2.0, -2.0}, square));
}

TEST_F(Tester, testGetPointsInsideSubPolygon)
{
  // Setup a steering velocity polygon
  setSteeringVelocityPolygonParameters(WHEELBASE, LOW_SPEED_THRESHOLD, {"slow"});
  // Use a polygon that covers (-0.3,-0.3) to (0.5,0.3)
  addSteeringAngleSubPolygon("slow", 0.0, 0.5, -0.5, 0.5, STEERING_POLYGON_SLOW_STR);
  createSteeringVelocityPolygon("limit");

  auto sub_polygons = velocity_polygon_->getSubPolygons();
  ASSERT_EQ(sub_polygons.size(), 1u);

  // Create collision points: some inside, some outside
  std::unordered_map<std::string, std::vector<nav2_collision_monitor::Point>> collision_map;
  collision_map["source"] = {
    {0.0, 0.0},     // inside
    {0.2, 0.1},     // inside
    {10.0, 10.0},   // outside
    {-10.0, -10.0}  // outside
  };

  int count = velocity_polygon_->callGetPointsInsideSubPolygon(sub_polygons[0], collision_map);
  EXPECT_EQ(count, 2);
}

TEST_F(Tester, testFindField)
{
  setSteeringVelocityPolygonParameters(WHEELBASE, LOW_SPEED_THRESHOLD, {"slow", "fast"});
  addSteeringAngleSubPolygon("slow", 0.0, 0.5, -0.5, 0.5, STEERING_POLYGON_SLOW_STR);
  addSteeringAngleSubPolygon("fast", 0.5, 1.0, -0.5, 0.5, STEERING_POLYGON_FAST_STR);
  createSteeringVelocityPolygon("limit");

  // Find field for 0.3 speed at 0 angle → slow
  auto field = velocity_polygon_->callFindField(0.3, 0.0);
  ASSERT_NE(field, nullptr);
  EXPECT_EQ(field->velocity_polygon_name_, "slow");

  // Find field for 0.7 speed at 0 angle → fast
  field = velocity_polygon_->callFindField(0.7, 0.0);
  ASSERT_NE(field, nullptr);
  EXPECT_EQ(field->velocity_polygon_name_, "fast");

  // Out of range speed
  field = velocity_polygon_->callFindField(1.5, 0.0);
  EXPECT_EQ(field, nullptr);

  // Out of range angle
  field = velocity_polygon_->callFindField(0.3, 1.0);
  EXPECT_EQ(field, nullptr);
}

TEST_F(Tester, testFindFieldsForAngle)
{
  setSteeringVelocityPolygonParameters(WHEELBASE, LOW_SPEED_THRESHOLD, {"slow", "fast"});
  addSteeringAngleSubPolygon("slow", 0.0, 0.5, -0.5, 0.5, STEERING_POLYGON_SLOW_STR);
  addSteeringAngleSubPolygon("fast", 0.5, 1.0, -0.5, 0.5, STEERING_POLYGON_FAST_STR);
  createSteeringVelocityPolygon("limit");

  // At angle 0, both forward fields should match, sorted by linear_min ascending
  auto fields = velocity_polygon_->callFindFieldsForAngle(0.0, true);
  ASSERT_EQ(fields.size(), 2u);
  EXPECT_EQ(fields[0]->velocity_polygon_name_, "slow");
  EXPECT_EQ(fields[1]->velocity_polygon_name_, "fast");

  // At angle 1.0 (outside range), no fields
  fields = velocity_polygon_->callFindFieldsForAngle(1.0, true);
  EXPECT_EQ(fields.size(), 0u);
}

// ==================== validateSteering tests ====================

TEST_F(Tester, testValidateSteeringDirectionReversalHighSpeed)
{
  // Setup: 2 speed fields with steering angle
  setSteeringVelocityPolygonParameters(WHEELBASE, LOW_SPEED_THRESHOLD, {"slow", "fast"});
  addSteeringAngleSubPolygon("slow", 0.0, 0.5, -0.5, 0.5, STEERING_POLYGON_SLOW_STR);
  addSteeringAngleSubPolygon("fast", 0.5, 1.0, -0.5, 0.5, STEERING_POLYGON_FAST_STR);
  createSteeringVelocityPolygon("limit");

  // Update polygon first so internal state is set
  nav2_collision_monitor::Velocity vel{0.3, 0.0, 0.0};
  velocity_polygon_->updatePolygon(vel);

  // Target: forward, Current: backward (speed crosses zero), |current| > threshold
  nav2_collision_monitor::Velocity cmd_vel{0.5, 0.0, 0.3};  // forward target with steering
  nav2_collision_monitor::Velocity odom_vel{-0.5, 0.0, 0.1};  // moving backward
  std::unordered_map<std::string, std::vector<nav2_collision_monitor::Point>> collision_map;
  collision_map["source"] = {};

  nav2_collision_monitor::Action action{
    nav2_collision_monitor::DO_NOTHING, cmd_vel, ""};

  bool modified = velocity_polygon_->validateSteering(cmd_vel, odom_vel, collision_map, action);
  EXPECT_TRUE(modified);
  // Should clamp tw to maintain current steering angle
  EXPECT_EQ(action.action_type, nav2_collision_monitor::LIMIT);
}

TEST_F(Tester, testValidateSteeringDirectionReversalLowSpeed)
{
  setSteeringVelocityPolygonParameters(WHEELBASE, LOW_SPEED_THRESHOLD, {"slow"});
  addSteeringAngleSubPolygon("slow", 0.0, 0.5, -0.5, 0.5, STEERING_POLYGON_SLOW_STR);
  createSteeringVelocityPolygon("limit");

  nav2_collision_monitor::Velocity vel{0.05, 0.0, 0.0};
  velocity_polygon_->updatePolygon(vel);

  // Target: forward, Current: backward but below threshold → allow steering freely
  nav2_collision_monitor::Velocity cmd_vel{0.3, 0.0, 0.1};
  nav2_collision_monitor::Velocity odom_vel{-0.05, 0.0, 0.0};  // below threshold
  std::unordered_map<std::string, std::vector<nav2_collision_monitor::Point>> collision_map;
  collision_map["source"] = {};

  nav2_collision_monitor::Action action{
    nav2_collision_monitor::DO_NOTHING, cmd_vel, ""};

  bool modified = velocity_polygon_->validateSteering(cmd_vel, odom_vel, collision_map, action);
  EXPECT_FALSE(modified);  // steering allowed freely
}

TEST_F(Tester, testValidateSteeringBothBelowThreshold)
{
  setSteeringVelocityPolygonParameters(WHEELBASE, LOW_SPEED_THRESHOLD, {"slow"});
  addSteeringAngleSubPolygon("slow", 0.0, 0.5, -0.5, 0.5, STEERING_POLYGON_SLOW_STR);
  createSteeringVelocityPolygon("limit");

  nav2_collision_monitor::Velocity vel{0.05, 0.0, 0.0};
  velocity_polygon_->updatePolygon(vel);

  // Both target and current below threshold → done (no modification)
  nav2_collision_monitor::Velocity cmd_vel{0.05, 0.0, 0.02};
  nav2_collision_monitor::Velocity odom_vel{0.05, 0.0, 0.01};
  std::unordered_map<std::string, std::vector<nav2_collision_monitor::Point>> collision_map;
  collision_map["source"] = {};

  nav2_collision_monitor::Action action{
    nav2_collision_monitor::DO_NOTHING, cmd_vel, ""};

  bool modified = velocity_polygon_->validateSteering(cmd_vel, odom_vel, collision_map, action);
  EXPECT_FALSE(modified);
}

TEST_F(Tester, testValidateSteeringSameBucketResultStaysInField)
{
  setSteeringVelocityPolygonParameters(WHEELBASE, LOW_SPEED_THRESHOLD, {"slow", "fast"});
  addSteeringAngleSubPolygon("slow", 0.0, 0.5, -0.5, 0.5, STEERING_POLYGON_SLOW_STR);
  addSteeringAngleSubPolygon("fast", 0.5, 1.0, -0.5, 0.5, STEERING_POLYGON_FAST_STR);
  createSteeringVelocityPolygon("limit");

  nav2_collision_monitor::Velocity vel{0.3, 0.0, 0.0};
  velocity_polygon_->updatePolygon(vel);

  // Same field, result velocity stays in same field, no collision check needed → no modification
  nav2_collision_monitor::Velocity cmd_vel{0.4, 0.0, 0.0};  // target in slow field
  nav2_collision_monitor::Velocity odom_vel{0.2, 0.0, 0.0};  // current in slow field
  std::unordered_map<std::string, std::vector<nav2_collision_monitor::Point>> collision_map;
  collision_map["source"] = {};  // no collision points

  nav2_collision_monitor::Action action{
    nav2_collision_monitor::DO_NOTHING, cmd_vel, ""};

  bool modified = velocity_polygon_->validateSteering(cmd_vel, odom_vel, collision_map, action);
  EXPECT_FALSE(modified);
}

TEST_F(Tester, testValidateSteeringSameBucketFasterFieldCollision)
{
  setSteeringVelocityPolygonParameters(WHEELBASE, LOW_SPEED_THRESHOLD, {"slow", "fast"});
  addSteeringAngleSubPolygon("slow", 0.0, 0.5, -0.5, 0.5, STEERING_POLYGON_SLOW_STR);
  addSteeringAngleSubPolygon("fast", 0.5, 1.0, -0.5, 0.5, STEERING_POLYGON_FAST_STR);
  createSteeringVelocityPolygon("limit");

  nav2_collision_monitor::Velocity vel{0.3, 0.0, 0.0};
  velocity_polygon_->updatePolygon(vel);

  // Same field (cmd_vel maps to slow field), but result velocity from prior processing
  // falls in the next faster (fast) field which has collision → limit speed.
  // The steering check sees result_vel is in a different field than current, checks
  // collision there, and caps speed to current field's linear_max.
  nav2_collision_monitor::Velocity cmd_vel{0.4, 0.0, 0.0};  // target in slow field
  nav2_collision_monitor::Velocity odom_vel{0.2, 0.0, 0.0};  // current in slow field
  std::unordered_map<std::string, std::vector<nav2_collision_monitor::Point>> collision_map;
  // Place collision points inside the fast polygon
  collision_map["source"] = {
    {1.2, 0.0},   // inside fast polygon
    {1.3, 0.1},   // inside fast polygon
  };

  // Set robot_action with a velocity that exceeds the slow field max
  // (simulating that the main loop allowed higher speed)
  nav2_collision_monitor::Velocity action_vel{0.6, 0.0, 0.0};
  nav2_collision_monitor::Action action{
    nav2_collision_monitor::DO_NOTHING, action_vel, ""};

  bool modified = velocity_polygon_->validateSteering(cmd_vel, odom_vel, collision_map, action);
  EXPECT_TRUE(modified);
  EXPECT_EQ(action.action_type, nav2_collision_monitor::LIMIT);
  // Speed should be limited to slow field's linear_max in baselink speed
  // At 0 steering angle, that's just 0.5
  EXPECT_LE(std::abs(action.req_vel.x), 0.5 + 1e-6);
}

TEST_F(Tester, testValidateSteeringDifferentBucketCollisionFree)
{
  // Fields at different steering angles
  setSteeringVelocityPolygonParameters(WHEELBASE, LOW_SPEED_THRESHOLD,
    {"straight_slow", "straight_fast", "left_slow", "left_fast"});
  addSteeringAngleSubPolygon("straight_slow", 0.0, 0.5, -0.1, 0.1, STEERING_POLYGON_SLOW_STR);
  addSteeringAngleSubPolygon("straight_fast", 0.5, 1.0, -0.1, 0.1, STEERING_POLYGON_FAST_STR);
  addSteeringAngleSubPolygon("left_slow", 0.0, 0.5, 0.1, 0.5, STEERING_POLYGON_SLOW_STR);
  addSteeringAngleSubPolygon("left_fast", 0.5, 1.0, 0.1, 0.5, STEERING_POLYGON_FAST_STR);
  createSteeringVelocityPolygon("limit");

  nav2_collision_monitor::Velocity vel{0.3, 0.0, 0.0};
  velocity_polygon_->updatePolygon(vel);

  // Current: straight at 0.3, Target: left at 0.3
  nav2_collision_monitor::Velocity odom_vel{0.3, 0.0, 0.0};  // straight
  double target_sa = 0.3;
  double target_tw = std::tan(target_sa) * 0.3 / WHEELBASE;
  nav2_collision_monitor::Velocity cmd_vel{0.3, 0.0, target_tw};

  std::unordered_map<std::string, std::vector<nav2_collision_monitor::Point>> collision_map;
  collision_map["source"] = {};  // no collision

  nav2_collision_monitor::Action action{
    nav2_collision_monitor::DO_NOTHING, cmd_vel, ""};

  bool modified = velocity_polygon_->validateSteering(cmd_vel, odom_vel, collision_map, action);
  EXPECT_FALSE(modified);  // fastest field is collision-free → done
}

TEST_F(Tester, testValidateSteeringDifferentBucketAllCollision)
{
  // Fields at different steering angles
  setSteeringVelocityPolygonParameters(WHEELBASE, LOW_SPEED_THRESHOLD,
    {"straight_slow", "left_slow"});
  addSteeringAngleSubPolygon("straight_slow", 0.0, 0.5, -0.1, 0.1, STEERING_POLYGON_SLOW_STR);
  addSteeringAngleSubPolygon("left_slow", 0.0, 0.5, 0.1, 0.5, STEERING_POLYGON_SLOW_STR);
  createSteeringVelocityPolygon("limit");

  nav2_collision_monitor::Velocity vel{0.3, 0.0, 0.0};
  velocity_polygon_->updatePolygon(vel);

  // Current: straight at 0.3, Target: left at 0.3
  nav2_collision_monitor::Velocity odom_vel{0.3, 0.0, 0.0};
  double target_sa = 0.3;
  double target_tw = std::tan(target_sa) * 0.3 / WHEELBASE;
  nav2_collision_monitor::Velocity cmd_vel{0.3, 0.0, target_tw};

  std::unordered_map<std::string, std::vector<nav2_collision_monitor::Point>> collision_map;
  // Collision in all left fields
  collision_map["source"] = {
    {0.0, 0.0},
    {0.2, 0.1},
  };

  nav2_collision_monitor::Action action{
    nav2_collision_monitor::DO_NOTHING, cmd_vel, ""};

  bool modified = velocity_polygon_->validateSteering(cmd_vel, odom_vel, collision_map, action);
  // Escape hatch: every left field is occupied, but the slowest field admits the
  // current speed (0.3 < 0.5), so steering into the occupied bucket is allowed
  // (the obstacle is already inside even the smallest field). Speed (0.3) is
  // within the slowest field's max (0.5), so nothing to change.
  EXPECT_FALSE(modified);
}

TEST_F(Tester, testValidateSteeringDifferentBucketAllCollisionTooFastHolds)
{
  // Escape hatch boundary: every left field is occupied AND the current speed
  // (0.7) exceeds even the slowest field's max (0.5). The robot must NOT steer
  // into the occupied bucket yet — it slows and holds at the current (straight)
  // bucket boundary first.
  setSteeringVelocityPolygonParameters(WHEELBASE, LOW_SPEED_THRESHOLD,
    {"straight_slow", "straight_fast", "left_slow"});
  addSteeringAngleSubPolygon("straight_slow", 0.0, 0.5, -0.1, 0.1, STEERING_POLYGON_SLOW_STR);
  addSteeringAngleSubPolygon("straight_fast", 0.5, 1.0, -0.1, 0.1, STEERING_POLYGON_FAST_STR);
  addSteeringAngleSubPolygon("left_slow", 0.0, 0.5, 0.1, 0.5, STEERING_POLYGON_SLOW_STR);
  createSteeringVelocityPolygon("limit");

  nav2_collision_monitor::Velocity vel{0.7, 0.0, 0.0};
  velocity_polygon_->updatePolygon(vel);

  nav2_collision_monitor::Velocity odom_vel{0.7, 0.0, 0.0};  // straight, fast
  double target_sa = 0.3;
  double target_tw = std::tan(target_sa) * 0.7 / WHEELBASE;
  nav2_collision_monitor::Velocity cmd_vel{0.7, 0.0, target_tw};

  std::unordered_map<std::string, std::vector<nav2_collision_monitor::Point>> collision_map;
  // Occupy the (slow-geometry) left field.
  collision_map["source"] = {
    {0.0, 0.0},
    {0.2, 0.1},
  };

  nav2_collision_monitor::Action action{
    nav2_collision_monitor::DO_NOTHING, cmd_vel, ""};

  bool modified = velocity_polygon_->validateSteering(cmd_vel, odom_vel, collision_map, action);
  EXPECT_TRUE(modified);
  double result_sa = velocity_polygon_->callComputeSteeringAngle(action.req_vel);
  EXPECT_NEAR(result_sa, 0.1 - 0.01, 0.03);  // held at straight boundary, not steered in
}

TEST_F(Tester, testValidateSteeringDifferentBucketDecelerationToValidField)
{
  // Setup: straight and left, each with slow and fast fields
  setSteeringVelocityPolygonParameters(WHEELBASE, LOW_SPEED_THRESHOLD,
    {"straight_slow", "straight_fast", "left_slow", "left_fast"});
  addSteeringAngleSubPolygon("straight_slow", 0.0, 0.5, -0.1, 0.1, STEERING_POLYGON_SLOW_STR);
  addSteeringAngleSubPolygon("straight_fast", 0.5, 1.0, -0.1, 0.1, STEERING_POLYGON_FAST_STR);
  addSteeringAngleSubPolygon("left_slow", 0.0, 0.5, 0.1, 0.5, STEERING_POLYGON_SLOW_STR);
  addSteeringAngleSubPolygon("left_fast", 0.5, 1.0, 0.1, 0.5, STEERING_POLYGON_FAST_STR);
  createSteeringVelocityPolygon("limit");

  nav2_collision_monitor::Velocity vel{0.7, 0.0, 0.0};
  velocity_polygon_->updatePolygon(vel);

  // Current: straight at 0.7 (fast field), Target: left at 0.7 (fast field)
  nav2_collision_monitor::Velocity odom_vel{0.7, 0.0, 0.0};
  double target_sa = 0.3;
  double target_tw = std::tan(target_sa) * 0.7 / WHEELBASE;
  nav2_collision_monitor::Velocity cmd_vel{0.7, 0.0, target_tw};

  std::unordered_map<std::string, std::vector<nav2_collision_monitor::Point>> collision_map;
  // Collision in the fast left polygon only (points inside the large polygon)
  collision_map["source"] = {
    {1.2, 0.5},
    {1.3, 0.6},
  };

  nav2_collision_monitor::Action action{
    nav2_collision_monitor::DO_NOTHING, cmd_vel, ""};

  bool modified = velocity_polygon_->validateSteering(cmd_vel, odom_vel, collision_map, action);
  EXPECT_TRUE(modified);
  EXPECT_EQ(action.action_type, nav2_collision_monitor::LIMIT);
  // Speed should be limited to slow field's linear_max (0.5) converted to baselink
  // At the neighbour bucket boundary (0.1 rad), baselink = 0.5 * cos(0.1)
  double neighbour_angle = 0.1;  // straight bucket boundary toward left
  double expected_max = 0.5 * std::cos(neighbour_angle);
  EXPECT_LE(std::abs(action.req_vel.x), expected_max + 1e-3);
}

TEST_F(Tester, testValidateSteeringBackwardSameBucketFasterFieldCollision)
{
  // Two backward fields in the same bucket:
  // backward_slow: [-0.3, 0.0], backward_mid: [-0.7, -0.3]
  setSteeringVelocityPolygonParameters(WHEELBASE, LOW_SPEED_THRESHOLD,
    {"backward_slow", "backward_mid"});
  addSteeringAngleSubPolygon("backward_slow", -0.3, 0.0, -0.5, 0.5, STEERING_POLYGON_SLOW_STR);
  addSteeringAngleSubPolygon("backward_mid", -0.7, -0.3, -0.5, 0.5, STEERING_POLYGON_FAST_STR);
  createSteeringVelocityPolygon("limit");

  nav2_collision_monitor::Velocity vel{-0.2, 0.0, 0.0};
  velocity_polygon_->updatePolygon(vel);

  // cmd_vel and odom both in backward_slow field
  nav2_collision_monitor::Velocity cmd_vel{-0.2, 0.0, 0.0};
  nav2_collision_monitor::Velocity odom_vel{-0.1, 0.0, 0.0};
  std::unordered_map<std::string, std::vector<nav2_collision_monitor::Point>> collision_map;
  // Collision points inside the backward_mid (larger) polygon
  collision_map["source"] = {
    {-0.6, 0.0},
    {-0.7, 0.1},
  };

  // result velocity from prior processing falls in backward_mid field
  nav2_collision_monitor::Velocity action_vel{-0.5, 0.0, 0.0};
  nav2_collision_monitor::Action action{
    nav2_collision_monitor::DO_NOTHING, action_vel, ""};

  bool modified = velocity_polygon_->validateSteering(cmd_vel, odom_vel, collision_map, action);
  EXPECT_TRUE(modified);
  EXPECT_EQ(action.action_type, nav2_collision_monitor::LIMIT);
  // Speed should be limited to backward_slow's linear_min (-0.3) converted to baselink
  // At 0 steering angle, that's -0.3
  EXPECT_GE(action.req_vel.x, -0.3 - 1e-6);  // not more negative than -0.3
}

TEST_F(Tester, testValidateSteeringStandstillSameBucketCapsToOccupiedBucketLimit)
{
  // amr47 2026-07-15 regression: robot at standstill, commanded hard reverse,
  // next-faster backward field occupied. The old standstill exemption passed
  // the full command through; the controller then overshot the physical
  // fieldset speed-bin boundary into the occupied field → protective-field
  // e-stop. The command must instead be capped to the slow field's bound.
  setSteeringVelocityPolygonParameters(WHEELBASE, LOW_SPEED_THRESHOLD,
    {"backward_slow", "backward_mid"});
  addSteeringAngleSubPolygon("backward_slow", -0.3, 0.0, -0.5, 0.5, STEERING_POLYGON_SLOW_STR);
  addSteeringAngleSubPolygon("backward_mid", -0.7, -0.3, -0.5, 0.5, STEERING_POLYGON_FAST_STR);
  createSteeringVelocityPolygon("limit");

  nav2_collision_monitor::Velocity cmd_vel{-0.6, 0.0, 0.0};
  nav2_collision_monitor::Velocity odom_vel{-0.005, 0.0, 0.0};  // standstill
  velocity_polygon_->updatePolygon(cmd_vel);

  std::unordered_map<std::string, std::vector<nav2_collision_monitor::Point>> collision_map;
  // Obstacle inside the backward_mid (larger) polygon only
  collision_map["source"] = {
    {-0.6, 0.0},
    {-0.7, 0.1},
  };

  nav2_collision_monitor::Action action{
    nav2_collision_monitor::DO_NOTHING, cmd_vel, ""};

  bool modified = velocity_polygon_->validateSteering(cmd_vel, odom_vel, collision_map, action);
  EXPECT_TRUE(modified);
  EXPECT_EQ(action.action_type, nav2_collision_monitor::LIMIT);
  // Capped just inside backward_slow ([-0.3, 0]) ...
  EXPECT_GE(action.req_vel.x, -0.3);
  // ... but the robot must still be able to start moving
  EXPECT_LE(action.req_vel.x, -0.25);
}

TEST_F(Tester, testValidateSteeringStandstillSameBucketNextFieldFreeAllowsMore)
{
  // Same standstill start, but the next-faster field is collision-free: the
  // startup cap rises to that field's bound and the command passes unmodified.
  setSteeringVelocityPolygonParameters(WHEELBASE, LOW_SPEED_THRESHOLD,
    {"backward_slow", "backward_mid"});
  addSteeringAngleSubPolygon("backward_slow", -0.3, 0.0, -0.5, 0.5, STEERING_POLYGON_SLOW_STR);
  addSteeringAngleSubPolygon("backward_mid", -0.7, -0.3, -0.5, 0.5, STEERING_POLYGON_FAST_STR);
  createSteeringVelocityPolygon("limit");

  nav2_collision_monitor::Velocity cmd_vel{-0.6, 0.0, 0.0};
  nav2_collision_monitor::Velocity odom_vel{-0.005, 0.0, 0.0};  // standstill
  velocity_polygon_->updatePolygon(cmd_vel);

  std::unordered_map<std::string, std::vector<nav2_collision_monitor::Point>> collision_map;
  collision_map["source"] = {};  // no obstacles

  nav2_collision_monitor::Action action{
    nav2_collision_monitor::DO_NOTHING, cmd_vel, ""};

  bool modified = velocity_polygon_->validateSteering(cmd_vel, odom_vel, collision_map, action);
  EXPECT_FALSE(modified);
  EXPECT_NEAR(action.req_vel.x, -0.6, 1e-6);
}

TEST_F(Tester, testValidateSteeringReversalLowSpeedCapsToOccupiedBucketLimit)
{
  // Direction reversal at low speed: steering is allowed freely, but the speed
  // must still respect the startup cap in the TARGET direction — the old code
  // passed the full command through here as well.
  setSteeringVelocityPolygonParameters(WHEELBASE, LOW_SPEED_THRESHOLD,
    {"backward_slow", "backward_mid"});
  addSteeringAngleSubPolygon("backward_slow", -0.3, 0.0, -0.5, 0.5, STEERING_POLYGON_SLOW_STR);
  addSteeringAngleSubPolygon("backward_mid", -0.7, -0.3, -0.5, 0.5, STEERING_POLYGON_FAST_STR);
  createSteeringVelocityPolygon("limit");

  nav2_collision_monitor::Velocity cmd_vel{-0.6, 0.0, 0.0};
  nav2_collision_monitor::Velocity odom_vel{0.05, 0.0, 0.0};  // creeping forward
  velocity_polygon_->updatePolygon(cmd_vel);

  std::unordered_map<std::string, std::vector<nav2_collision_monitor::Point>> collision_map;
  // Obstacle inside the backward_mid (larger) polygon only
  collision_map["source"] = {
    {-0.6, 0.0},
    {-0.7, 0.1},
  };

  nav2_collision_monitor::Action action{
    nav2_collision_monitor::DO_NOTHING, cmd_vel, ""};

  bool modified = velocity_polygon_->validateSteering(cmd_vel, odom_vel, collision_map, action);
  EXPECT_TRUE(modified);
  EXPECT_EQ(action.action_type, nav2_collision_monitor::LIMIT);
  EXPECT_GE(action.req_vel.x, -0.3);
  EXPECT_LE(action.req_vel.x, -0.25);
}

TEST_F(Tester, testValidateSteeringNotApplicableToNonSteering)
{
  // Create a normal theta-based velocity polygon (not steering angle)
  createVelocityPolygon("stop", IS_NOT_HOLONOMIC);

  nav2_collision_monitor::Velocity cmd_vel{0.3, 0.0, 0.0};
  nav2_collision_monitor::Velocity odom_vel{0.2, 0.0, 0.0};
  std::unordered_map<std::string, std::vector<nav2_collision_monitor::Point>> collision_map;
  collision_map["source"] = {};

  nav2_collision_monitor::Action action{
    nav2_collision_monitor::DO_NOTHING, cmd_vel, ""};

  bool modified = velocity_polygon_->validateSteering(cmd_vel, odom_vel, collision_map, action);
  EXPECT_FALSE(modified);  // Should not apply to theta-based polygons
}

// ==================== validateSteering different-bucket exceedance fix tests ====================

TEST_F(Tester, testValidateSteeringDifferentBucketExceedsFastestField)
{
  // straight: slow[0,0.5] + fast[0.5,1.0] sa[-0.1,0.1]
  // left: slow[0,0.5] only sa[0.1,0.5]
  // When steering from straight into left at speed 0.7, which exceeds left's max (0.5),
  // speed should be clamped even though left is collision-free.
  setSteeringVelocityPolygonParameters(WHEELBASE, LOW_SPEED_THRESHOLD,
    {"straight_slow", "straight_fast", "left_slow"});
  addSteeringAngleSubPolygon("straight_slow", 0.0, 0.5, -0.1, 0.1, STEERING_POLYGON_SLOW_STR);
  addSteeringAngleSubPolygon("straight_fast", 0.5, 1.0, -0.1, 0.1, STEERING_POLYGON_FAST_STR);
  addSteeringAngleSubPolygon("left_slow", 0.0, 0.5, 0.1, 0.5, STEERING_POLYGON_SLOW_STR);
  createSteeringVelocityPolygon("limit");

  nav2_collision_monitor::Velocity vel{0.7, 0.0, 0.0};
  velocity_polygon_->updatePolygon(vel);

  // Current: straight at 0.7 (fast field), Target: left at 0.7
  nav2_collision_monitor::Velocity odom_vel{0.7, 0.0, 0.0};
  double target_sa = 0.3;
  double target_tw = std::tan(target_sa) * 0.7 / WHEELBASE;
  nav2_collision_monitor::Velocity cmd_vel{0.7, 0.0, target_tw};

  std::unordered_map<std::string, std::vector<nav2_collision_monitor::Point>> collision_map;
  collision_map["source"] = {};  // no collisions

  nav2_collision_monitor::Action action{
    nav2_collision_monitor::DO_NOTHING, cmd_vel, ""};

  bool modified = velocity_polygon_->validateSteering(cmd_vel, odom_vel, collision_map, action);
  EXPECT_TRUE(modified);
  EXPECT_EQ(action.action_type, nav2_collision_monitor::LIMIT);
  // Speed should be clamped to left_slow's max (0.5) at neighbour boundary (0.1 rad)
  double neighbour_angle = 0.1;
  double expected_max = 0.5 * std::cos(neighbour_angle);
  EXPECT_LE(std::abs(action.req_vel.x), expected_max + 1e-3);
}

// ==================== clampToMaxField tests ====================

TEST_F(Tester, testClampToMaxFieldNoClamping)
{
  // slow[0,0.5] + fast[0.5,1.0] sa[-0.5,0.5]
  // phys_sa=0, cmd=0.8 → within 1.0 max, no clamping
  setSteeringVelocityPolygonParameters(WHEELBASE, LOW_SPEED_THRESHOLD, {"slow", "fast"});
  addSteeringAngleSubPolygon("slow", 0.0, 0.5, -0.5, 0.5, STEERING_POLYGON_SLOW_STR);
  addSteeringAngleSubPolygon("fast", 0.5, 1.0, -0.5, 0.5, STEERING_POLYGON_FAST_STR);
  createSteeringVelocityPolygon("limit");

  nav2_collision_monitor::Velocity odom_vel{0.8, 0.0, 0.0};  // phys_sa=0
  nav2_collision_monitor::Velocity cmd_vel{0.8, 0.0, 0.0};
  nav2_collision_monitor::Action action{
    nav2_collision_monitor::DO_NOTHING, cmd_vel, ""};

  bool modified = velocity_polygon_->callClampToMaxField(odom_vel, action);
  EXPECT_FALSE(modified);
}

TEST_F(Tester, testClampToMaxFieldExceedsMax)
{
  // slow[0,0.5] only sa[-0.5,0.5]
  // phys_sa=0, cmd=0.8 → exceeds 0.5, should clamp
  setSteeringVelocityPolygonParameters(WHEELBASE, LOW_SPEED_THRESHOLD, {"slow"});
  addSteeringAngleSubPolygon("slow", 0.0, 0.5, -0.5, 0.5, STEERING_POLYGON_SLOW_STR);
  createSteeringVelocityPolygon("limit");

  nav2_collision_monitor::Velocity odom_vel{0.3, 0.0, 0.0};  // phys_sa=0
  nav2_collision_monitor::Velocity cmd_vel{0.8, 0.0, 0.0};
  nav2_collision_monitor::Action action{
    nav2_collision_monitor::DO_NOTHING, cmd_vel, ""};

  bool modified = velocity_polygon_->callClampToMaxField(odom_vel, action);
  EXPECT_TRUE(modified);
  EXPECT_EQ(action.action_type, nav2_collision_monitor::LIMIT);
  // Clamped to 0.5 at sa=0 → baselink = 0.5 * cos(0) = 0.5
  EXPECT_LE(std::abs(action.req_vel.x), 0.5 + 1e-6);
}

TEST_F(Tester, testClampToMaxFieldTurnedBucketLower)
{
  // straight: slow[0,0.5] + fast[0.5,1.0] sa[-0.1,0.1]
  // turned: slow[0,0.5] only sa[0.3,0.6]
  // phys_sa=0.4 (turned bucket), cmd=0.8 → exceeds turned max (0.5), clamp
  setSteeringVelocityPolygonParameters(WHEELBASE, LOW_SPEED_THRESHOLD,
    {"straight_slow", "straight_fast", "turned_slow"});
  addSteeringAngleSubPolygon("straight_slow", 0.0, 0.5, -0.1, 0.1, STEERING_POLYGON_SLOW_STR);
  addSteeringAngleSubPolygon("straight_fast", 0.5, 1.0, -0.1, 0.1, STEERING_POLYGON_FAST_STR);
  addSteeringAngleSubPolygon("turned_slow", 0.0, 0.5, 0.3, 0.6, STEERING_POLYGON_SLOW_STR);
  createSteeringVelocityPolygon("limit");

  double phys_sa = 0.4;
  double phys_tw = std::tan(phys_sa) * 0.5 / WHEELBASE;
  nav2_collision_monitor::Velocity odom_vel{0.5, 0.0, phys_tw};  // phys_sa=0.4

  // cmd targets some steering angle at 0.8 speed
  double cmd_sa = 0.2;
  double cmd_tw = std::tan(cmd_sa) * 0.8 / WHEELBASE;
  nav2_collision_monitor::Velocity cmd_vel{0.8, 0.0, cmd_tw};
  nav2_collision_monitor::Action action{
    nav2_collision_monitor::DO_NOTHING, cmd_vel, ""};

  bool modified = velocity_polygon_->callClampToMaxField(odom_vel, action);
  EXPECT_TRUE(modified);
  EXPECT_EQ(action.action_type, nav2_collision_monitor::LIMIT);
  // Max sw speed at physical angle (0.4) is 0.5, clamped baselink = 0.5 * cos(cmd_sa)
  double expected_max_baselink = 0.5 * std::cos(cmd_sa);
  EXPECT_LE(std::abs(action.req_vel.x), expected_max_baselink + 1e-3);
}

TEST_F(Tester, testClampToMaxFieldRule2NotYetInBucket)
{
  // Rule 2: turned→straight transition, still physically turned
  // turned[0.3,0.6]: slow[0,0.5]; straight[-0.1,0.1]: slow[0,0.5]+fast[0.5,1.0]
  // phys_sa=0.4 (turned), cmd targets straight at 0.8 → clamp to turned max (0.5)
  setSteeringVelocityPolygonParameters(WHEELBASE, LOW_SPEED_THRESHOLD,
    {"turned_slow", "straight_slow", "straight_fast"});
  addSteeringAngleSubPolygon("turned_slow", 0.0, 0.5, 0.3, 0.6, STEERING_POLYGON_SLOW_STR);
  addSteeringAngleSubPolygon("straight_slow", 0.0, 0.5, -0.1, 0.1, STEERING_POLYGON_SLOW_STR);
  addSteeringAngleSubPolygon("straight_fast", 0.5, 1.0, -0.1, 0.1, STEERING_POLYGON_FAST_STR);
  createSteeringVelocityPolygon("limit");

  double phys_sa = 0.4;
  double phys_tw = std::tan(phys_sa) * 0.3 / WHEELBASE;
  nav2_collision_monitor::Velocity odom_vel{0.3, 0.0, phys_tw};  // phys in turned bucket

  // Cmd targets straight at 0.8
  nav2_collision_monitor::Velocity cmd_vel{0.8, 0.0, 0.0};
  nav2_collision_monitor::Action action{
    nav2_collision_monitor::DO_NOTHING, cmd_vel, ""};

  bool modified = velocity_polygon_->callClampToMaxField(odom_vel, action);
  EXPECT_TRUE(modified);
  EXPECT_EQ(action.action_type, nav2_collision_monitor::LIMIT);
  // Max at phys_sa=0.4 is turned_slow max = 0.5
  // cmd_sa=0, so clamped baselink = 0.5 * cos(0) = 0.5
  EXPECT_LE(std::abs(action.req_vel.x), 0.5 + 1e-3);
}

TEST_F(Tester, testClampToMaxFieldRule2Arrived)
{
  // Same setup as above but phys_sa=0.0 (arrived in straight bucket)
  // cmd=0.8 → within straight fast max (1.0), no clamping
  setSteeringVelocityPolygonParameters(WHEELBASE, LOW_SPEED_THRESHOLD,
    {"turned_slow", "straight_slow", "straight_fast"});
  addSteeringAngleSubPolygon("turned_slow", 0.0, 0.5, 0.3, 0.6, STEERING_POLYGON_SLOW_STR);
  addSteeringAngleSubPolygon("straight_slow", 0.0, 0.5, -0.1, 0.1, STEERING_POLYGON_SLOW_STR);
  addSteeringAngleSubPolygon("straight_fast", 0.5, 1.0, -0.1, 0.1, STEERING_POLYGON_FAST_STR);
  createSteeringVelocityPolygon("limit");

  nav2_collision_monitor::Velocity odom_vel{0.8, 0.0, 0.0};  // phys_sa=0, in straight bucket

  nav2_collision_monitor::Velocity cmd_vel{0.8, 0.0, 0.0};
  nav2_collision_monitor::Action action{
    nav2_collision_monitor::DO_NOTHING, cmd_vel, ""};

  bool modified = velocity_polygon_->callClampToMaxField(odom_vel, action);
  EXPECT_FALSE(modified);  // Within fast field max, no clamping needed
}

TEST_F(Tester, testClampToMaxFieldBackward)
{
  // backward[-0.5,0] sa[-0.5,0.5]
  // phys_sa=0, cmd.x=-0.8 → exceeds -0.5, should clamp
  setSteeringVelocityPolygonParameters(WHEELBASE, LOW_SPEED_THRESHOLD, {"backward"});
  addSteeringAngleSubPolygon("backward", -0.5, 0.0, -0.5, 0.5, STEERING_POLYGON_SLOW_STR);
  createSteeringVelocityPolygon("limit");

  nav2_collision_monitor::Velocity odom_vel{-0.3, 0.0, 0.0};  // phys_sa=0
  nav2_collision_monitor::Velocity cmd_vel{-0.8, 0.0, 0.0};
  nav2_collision_monitor::Action action{
    nav2_collision_monitor::DO_NOTHING, cmd_vel, ""};

  bool modified = velocity_polygon_->callClampToMaxField(odom_vel, action);
  EXPECT_TRUE(modified);
  EXPECT_EQ(action.action_type, nav2_collision_monitor::LIMIT);
  // Clamped to -0.5 at sa=0 → baselink = -0.5 * cos(0) = -0.5
  EXPECT_GE(action.req_vel.x, -0.5 - 1e-6);
}

TEST_F(Tester, testClampToMaxFieldNoFieldsZeros)
{
  // Fields only at sa[0.3,0.6]
  // phys_sa=0.0 → no fields found, velocity should be zeroed
  setSteeringVelocityPolygonParameters(WHEELBASE, LOW_SPEED_THRESHOLD, {"turned_only"});
  addSteeringAngleSubPolygon("turned_only", 0.0, 0.5, 0.3, 0.6, STEERING_POLYGON_SLOW_STR);
  createSteeringVelocityPolygon("limit");

  nav2_collision_monitor::Velocity odom_vel{0.3, 0.0, 0.0};  // phys_sa=0
  nav2_collision_monitor::Velocity cmd_vel{0.3, 0.0, 0.0};
  nav2_collision_monitor::Action action{
    nav2_collision_monitor::DO_NOTHING, cmd_vel, ""};

  bool modified = velocity_polygon_->callClampToMaxField(odom_vel, action);
  EXPECT_TRUE(modified);
  EXPECT_EQ(action.action_type, nav2_collision_monitor::LIMIT);
  EXPECT_NEAR(action.req_vel.x, 0.0, 1e-6);
  EXPECT_NEAR(action.req_vel.tw, 0.0, 1e-6);
}

TEST_F(Tester, testClampToMaxFieldNonSteeringSkipped)
{
  // Create a normal theta-based velocity polygon (not steering angle)
  createVelocityPolygon("stop", IS_NOT_HOLONOMIC);

  nav2_collision_monitor::Velocity odom_vel{0.3, 0.0, 0.0};
  nav2_collision_monitor::Velocity cmd_vel{0.3, 0.0, 0.0};
  nav2_collision_monitor::Action action{
    nav2_collision_monitor::DO_NOTHING, cmd_vel, ""};

  bool modified = velocity_polygon_->callClampToMaxField(odom_vel, action);
  EXPECT_FALSE(modified);  // Should not apply to theta-based polygons
}

TEST_F(Tester, testClampToMaxFieldPreservesDirection)
{
  // slow[0,0.5] sa[-0.5,0.5]
  // phys_sa=0, cmd x=0.8 tw=0.3 → speed clamped, tw adjusted to same steering angle
  setSteeringVelocityPolygonParameters(WHEELBASE, LOW_SPEED_THRESHOLD, {"slow"});
  addSteeringAngleSubPolygon("slow", 0.0, 0.5, -0.5, 0.5, STEERING_POLYGON_SLOW_STR);
  createSteeringVelocityPolygon("limit");

  nav2_collision_monitor::Velocity odom_vel{0.3, 0.0, 0.0};  // phys_sa=0
  double cmd_sa_input = 0.2;
  double cmd_tw_input = std::tan(cmd_sa_input) * 0.8 / WHEELBASE;
  nav2_collision_monitor::Velocity cmd_vel{0.8, 0.0, cmd_tw_input};
  nav2_collision_monitor::Action action{
    nav2_collision_monitor::DO_NOTHING, cmd_vel, ""};

  bool modified = velocity_polygon_->callClampToMaxField(odom_vel, action);
  EXPECT_TRUE(modified);
  EXPECT_EQ(action.action_type, nav2_collision_monitor::LIMIT);
  // Speed clamped, steering angle should be preserved
  double clamped_sa = velocity_polygon_->callComputeSteeringAngle(action.req_vel);
  EXPECT_NEAR(clamped_sa, cmd_sa_input, 0.05);
  // Speed should be limited
  EXPECT_LE(std::abs(action.req_vel.x), 0.5 + 1e-3);
}


TEST_F(Tester, testValidateSteeringSameBucket90DegLimitsTw)
{
  // Two speed tiers at 90° steering angle range.
  // At 90° steering the robot is doing pure rotation (x≈0, tw carries all speed).
  // The old code compared result_vel.x against max_baselink (both ≈0 at 90°)
  // and never triggered a limit. The fix compares steering wheel speeds and
  // decomposes via sin/cos so that tw is correctly limited.
  setSteeringVelocityPolygonParameters(WHEELBASE, LOW_SPEED_THRESHOLD, {"slow_turn", "fast_turn"});
  // steering_angle range [1.0, 1.571] covers 90° (π/2 ≈ 1.5708)
  addSteeringAngleSubPolygon("slow_turn", 0.0, 0.3, 1.0, 1.571, STEERING_POLYGON_SLOW_STR);
  addSteeringAngleSubPolygon("fast_turn", 0.3, 1.0, 1.0, 1.571, STEERING_POLYGON_FAST_STR);
  createSteeringVelocityPolygon("limit");

  // updatePolygon with a velocity that maps to the slow field at ~90°
  // Pure rotation: x=0, tw=0.2 → sw_speed = hypot(0, 1.0*0.2) = 0.2 (in slow field)
  nav2_collision_monitor::Velocity vel{0.0, 0.0, 0.2};
  velocity_polygon_->updatePolygon(vel);

  // cmd_vel: pure rotation, steering angle = π/2
  // x=0, tw=0.5 → sw_speed = hypot(0, 1.0*0.5) = 0.5 (in fast field)
  nav2_collision_monitor::Velocity cmd_vel{0.0, 0.0, 0.5};
  // odom: currently in slow field
  nav2_collision_monitor::Velocity odom_vel{0.0, 0.0, 0.15};

  // Place collision points inside the fast polygon
  std::unordered_map<std::string, std::vector<nav2_collision_monitor::Point>> collision_map;
  collision_map["source"] = {
    {1.0, 0.0},
    {1.2, 0.1},
  };

  // robot_action with result velocity in the fast field (sw_speed = 0.5)
  nav2_collision_monitor::Velocity action_vel{0.0, 0.0, 0.5};
  nav2_collision_monitor::Action action{
    nav2_collision_monitor::DO_NOTHING, action_vel, ""};

  bool modified = velocity_polygon_->validateSteering(cmd_vel, odom_vel, collision_map, action);
  EXPECT_TRUE(modified);
  EXPECT_EQ(action.action_type, nav2_collision_monitor::LIMIT);

  // At 90° steering, x should be ~0 and tw should be limited
  EXPECT_NEAR(action.req_vel.x, 0.0, 1e-6);
  // tw should be limited to slow field's linear_max (0.3) decomposed at 90°:
  // limited_tw = limit_sw * sin(π/2) / wheelbase = 0.3 * 1.0 / 1.0 = 0.3
  EXPECT_LE(std::abs(action.req_vel.tw), 0.3 + 1e-6);
  EXPECT_GT(std::abs(action.req_vel.tw), 0.0);  // tw should not be zeroed

  // Verify the steering wheel speed of the result doesn't exceed the slow field max
  double result_sw = velocity_polygon_->callBaselinkToSteeringSpeed(
    action.req_vel.x, action.req_vel.tw);
  EXPECT_LE(std::abs(result_sw), 0.3 + 1e-6);
}

// ==================== Step 2 current-bucket limit enforced across buckets ====================

TEST_F(Tester, testValidateSteeringDifferentBucketCurrentBucketLimitEnforced)
{
  // Setup: straight has slow[0,0.5]+fast[0.5,1.0]; left has slow[0,0.5]+fast[0.5,1.0]
  // Robot is in straight_slow at 0.3. Next field (straight_fast) is collision-free →
  // current bucket allows up to 1.0. Neighbour (left) valid field allows up to 1.0.
  // min(1.0, 1.0) = 1.0 → no speed clamping needed.
  //
  // Now place collision in straight_fast → current bucket allows only 0.5.
  // Neighbour left_fast is collision-free → valid field allows 1.0.
  // min(0.5, 1.0) = 0.5 → speed must be clamped to 0.5 even though left allows more.
  setSteeringVelocityPolygonParameters(WHEELBASE, LOW_SPEED_THRESHOLD,
    {"straight_slow", "straight_fast", "left_slow", "left_fast"});
  addSteeringAngleSubPolygon("straight_slow", 0.0, 0.5, -0.1, 0.1, STEERING_POLYGON_SLOW_STR);
  addSteeringAngleSubPolygon("straight_fast", 0.5, 1.0, -0.1, 0.1, STEERING_POLYGON_FAST_STR);
  addSteeringAngleSubPolygon("left_slow", 0.0, 0.5, 0.1, 0.5, STEERING_POLYGON_SLOW_STR);
  addSteeringAngleSubPolygon("left_fast", 0.5, 1.0, 0.1, 0.5, STEERING_POLYGON_FAST_STR);
  createSteeringVelocityPolygon("limit");

  nav2_collision_monitor::Velocity vel{0.3, 0.0, 0.0};
  velocity_polygon_->updatePolygon(vel);

  // Current: straight at 0.3 (slow field), Target: left at 0.7
  nav2_collision_monitor::Velocity odom_vel{0.3, 0.0, 0.0};
  double target_sa = 0.3;
  double target_tw = std::tan(target_sa) * 0.7 / WHEELBASE;
  nav2_collision_monitor::Velocity cmd_vel{0.7, 0.0, target_tw};

  std::unordered_map<std::string, std::vector<nav2_collision_monitor::Point>> collision_map;
  // Collision points inside straight_fast polygon (the larger polygon)
  collision_map["source"] = {
    {1.2, 0.0},
    {1.3, 0.1},
  };

  nav2_collision_monitor::Action action{
    nav2_collision_monitor::DO_NOTHING, cmd_vel, ""};

  bool modified = velocity_polygon_->validateSteering(cmd_vel, odom_vel, collision_map, action);
  EXPECT_TRUE(modified);
  EXPECT_EQ(action.action_type, nav2_collision_monitor::LIMIT);
  // Current bucket limit is straight_slow max = 0.5 (because straight_fast has collision).
  // Neighbour valid field (left_fast) allows 1.0. min(0.5, 1.0) = 0.5.
  // At neighbour_angle = 0.1, baselink = 0.5 * cos(0.1)
  double neighbour_angle = 0.1;
  double expected_max = 0.5 * std::cos(neighbour_angle);
  EXPECT_LE(std::abs(action.req_vel.x), expected_max + 1e-3);
}

TEST_F(Tester, testValidateSteeringDifferentBucketCurrentBucketNoCollisionAllowsMore)
{
  // Same setup but no collision in straight_fast → current bucket allows up to 1.0.
  // Neighbour left has only slow[0,0.5] → valid field allows 0.5.
  // min(1.0, 0.5) = 0.5 → speed clamped by neighbour, not current bucket.
  setSteeringVelocityPolygonParameters(WHEELBASE, LOW_SPEED_THRESHOLD,
    {"straight_slow", "straight_fast", "left_slow"});
  addSteeringAngleSubPolygon("straight_slow", 0.0, 0.5, -0.1, 0.1, STEERING_POLYGON_SLOW_STR);
  addSteeringAngleSubPolygon("straight_fast", 0.5, 1.0, -0.1, 0.1, STEERING_POLYGON_FAST_STR);
  addSteeringAngleSubPolygon("left_slow", 0.0, 0.5, 0.1, 0.5, STEERING_POLYGON_SLOW_STR);
  createSteeringVelocityPolygon("limit");

  nav2_collision_monitor::Velocity vel{0.3, 0.0, 0.0};
  velocity_polygon_->updatePolygon(vel);

  // Current: straight at 0.3 (slow field), Target: left at 0.7
  nav2_collision_monitor::Velocity odom_vel{0.3, 0.0, 0.0};
  double target_sa = 0.3;
  double target_tw = std::tan(target_sa) * 0.7 / WHEELBASE;
  nav2_collision_monitor::Velocity cmd_vel{0.7, 0.0, target_tw};

  std::unordered_map<std::string, std::vector<nav2_collision_monitor::Point>> collision_map;
  collision_map["source"] = {};  // no collision

  nav2_collision_monitor::Action action{
    nav2_collision_monitor::DO_NOTHING, cmd_vel, ""};

  bool modified = velocity_polygon_->validateSteering(cmd_vel, odom_vel, collision_map, action);
  EXPECT_TRUE(modified);
  EXPECT_EQ(action.action_type, nav2_collision_monitor::LIMIT);
  // Current bucket limit = 1.0 (next field collision-free). Neighbour valid = 0.5.
  // min(1.0, 0.5) = 0.5 at neighbour_angle = 0.1
  double neighbour_angle = 0.1;
  double expected_max = 0.5 * std::cos(neighbour_angle);
  EXPECT_LE(std::abs(action.req_vel.x), expected_max + 1e-3);
}

TEST_F(Tester, testValidateSteeringDifferentBucketBothLimitsApply)
{
  // Current bucket has collision in next field → bucket limit = 0.5
  // Neighbour bucket valid field also has max 0.5 (only slow field, collision-free)
  // min(0.5, 0.5) = 0.5 → both limits agree, speed clamped
  setSteeringVelocityPolygonParameters(WHEELBASE, LOW_SPEED_THRESHOLD,
    {"straight_slow", "straight_fast", "left_slow"});
  addSteeringAngleSubPolygon("straight_slow", 0.0, 0.5, -0.1, 0.1, STEERING_POLYGON_SLOW_STR);
  addSteeringAngleSubPolygon("straight_fast", 0.5, 1.0, -0.1, 0.1, STEERING_POLYGON_FAST_STR);
  addSteeringAngleSubPolygon("left_slow", 0.0, 0.5, 0.1, 0.5, STEERING_POLYGON_SLOW_STR);
  createSteeringVelocityPolygon("limit");

  nav2_collision_monitor::Velocity vel{0.3, 0.0, 0.0};
  velocity_polygon_->updatePolygon(vel);

  nav2_collision_monitor::Velocity odom_vel{0.3, 0.0, 0.0};
  double target_sa = 0.3;
  double target_tw = std::tan(target_sa) * 0.7 / WHEELBASE;
  nav2_collision_monitor::Velocity cmd_vel{0.7, 0.0, target_tw};

  std::unordered_map<std::string, std::vector<nav2_collision_monitor::Point>> collision_map;
  // Collision in straight_fast
  collision_map["source"] = {
    {1.2, 0.0},
    {1.3, 0.1},
  };

  nav2_collision_monitor::Action action{
    nav2_collision_monitor::DO_NOTHING, cmd_vel, ""};

  bool modified = velocity_polygon_->validateSteering(cmd_vel, odom_vel, collision_map, action);
  EXPECT_TRUE(modified);
  EXPECT_EQ(action.action_type, nav2_collision_monitor::LIMIT);
  double neighbour_angle = 0.1;
  double expected_max = 0.5 * std::cos(neighbour_angle);
  EXPECT_LE(std::abs(action.req_vel.x), expected_max + 1e-3);
}

TEST_F(Tester, testValidateSteeringBackwardDifferentBucketCurrentBucketLimitEnforced)
{
  // Backward driving: straight has backward_slow[-0.5,0]+backward_fast[-1.0,-0.5]
  // Left has backward_slow[-0.5,0] only.
  // Robot at backward_slow. Collision in backward_fast → current bucket limit = -0.5.
  // Neighbour left backward_slow allows -0.5. min(0.5, 0.5) = 0.5.
  setSteeringVelocityPolygonParameters(WHEELBASE, LOW_SPEED_THRESHOLD,
    {"straight_bw_slow", "straight_bw_fast", "left_bw_slow"});
  addSteeringAngleSubPolygon(
    "straight_bw_slow", -0.5, 0.0, -0.1, 0.1, STEERING_POLYGON_SLOW_STR);
  addSteeringAngleSubPolygon(
    "straight_bw_fast", -1.0, -0.5, -0.1, 0.1, STEERING_POLYGON_FAST_STR);
  addSteeringAngleSubPolygon(
    "left_bw_slow", -0.5, 0.0, 0.1, 0.5, STEERING_POLYGON_SLOW_STR);
  createSteeringVelocityPolygon("limit");

  nav2_collision_monitor::Velocity vel{-0.3, 0.0, 0.0};
  velocity_polygon_->updatePolygon(vel);

  // Current: straight backward at -0.3, Target: left backward at -0.7
  nav2_collision_monitor::Velocity odom_vel{-0.3, 0.0, 0.0};
  double target_sa = 0.3;
  double target_tw = std::tan(target_sa) * std::abs(-0.7) / WHEELBASE;
  // For backward driving, tw sign is inverted
  target_tw = -target_tw;
  nav2_collision_monitor::Velocity cmd_vel{-0.7, 0.0, target_tw};

  std::unordered_map<std::string, std::vector<nav2_collision_monitor::Point>> collision_map;
  // Collision in backward_fast polygon
  collision_map["source"] = {
    {-0.6, 0.0},
    {-0.7, 0.1},
  };

  nav2_collision_monitor::Action action{
    nav2_collision_monitor::DO_NOTHING, cmd_vel, ""};

  bool modified = velocity_polygon_->validateSteering(cmd_vel, odom_vel, collision_map, action);
  EXPECT_TRUE(modified);
  EXPECT_EQ(action.action_type, nav2_collision_monitor::LIMIT);
  // Speed should be limited: current bucket limit is -0.5 (backward_fast has collision).
  // At neighbour angle boundary, baselink = -0.5 * cos(0.1)
  double neighbour_angle = 0.1;
  double expected_min = -0.5 * std::cos(neighbour_angle);
  EXPECT_GE(action.req_vel.x, expected_min - 1e-3);  // not more negative
}

// ==================== Step 6b: do not proceed into next bucket if speed not valid ====================

TEST_F(Tester, testValidateSteeringDifferentBucketHoldsCurrentAngleWhenTooFast)
{
  // Step 6b: when current_sw exceeds the neighbour's valid field max, the robot
  // must hold its current steering angle (not steer toward the boundary) and
  // decelerate first. Only after current_sw drops below the limit will 6b stop
  // firing and allow steering toward the next bucket.
  //
  // Setup: straight has slow[0,0.5]+fast[0.5,1.0], left has only slow[0,0.3]
  // The left bucket's max is 0.3 — lower than the robot's current speed (0.4).
  setSteeringVelocityPolygonParameters(WHEELBASE, LOW_SPEED_THRESHOLD,
    {"straight_slow", "straight_fast", "left_slow"});
  addSteeringAngleSubPolygon("straight_slow", 0.0, 0.5, -0.1, 0.1, STEERING_POLYGON_SLOW_STR);
  addSteeringAngleSubPolygon("straight_fast", 0.5, 1.0, -0.1, 0.1, STEERING_POLYGON_FAST_STR);
  addSteeringAngleSubPolygon("left_slow", 0.0, 0.3, 0.1, 0.5, STEERING_POLYGON_SLOW_STR);
  createSteeringVelocityPolygon("limit");

  nav2_collision_monitor::Velocity vel{0.4, 0.0, 0.0};
  velocity_polygon_->updatePolygon(vel);

  // Current: straight at 0.4 (sa=0), Target: left at 0.4
  nav2_collision_monitor::Velocity odom_vel{0.4, 0.0, 0.0};  // current_sa = 0
  double target_sa = 0.3;
  double target_tw = std::tan(target_sa) * 0.4 / WHEELBASE;
  nav2_collision_monitor::Velocity cmd_vel{0.4, 0.0, target_tw};

  std::unordered_map<std::string, std::vector<nav2_collision_monitor::Point>> collision_map;
  collision_map["source"] = {};  // no collisions

  // Pre-limited action velocity below the effective limit so 6a does NOT trigger.
  double pre_limited_sa = 0.3;
  double pre_limited_tw = std::tan(pre_limited_sa) * 0.2 / WHEELBASE;
  nav2_collision_monitor::Velocity action_vel{0.2, 0.0, pre_limited_tw};
  nav2_collision_monitor::Action action{
    nav2_collision_monitor::DO_NOTHING, action_vel, ""};

  bool modified = velocity_polygon_->validateSteering(cmd_vel, odom_vel, collision_map, action);
  EXPECT_TRUE(modified);
  EXPECT_EQ(action.action_type, nav2_collision_monitor::LIMIT);

  // Per README step 6b: "limit steering angle to boundary of current bucket",
  // inset by kAngleMargin (0.01 rad) to stay inside the field.
  // The current bucket (straight_slow) has steering_angle_max = 0.1, so the
  // steering angle is clamped to 0.1 - 0.01 = 0.09.
  double kAngleMargin = 0.01;
  double expected_boundary_sa = 0.1 - kAngleMargin;
  double result_sa = velocity_polygon_->callComputeSteeringAngle(action.req_vel);
  EXPECT_NEAR(result_sa, expected_boundary_sa, 0.02);

  // Speed: result_sw was not clamped (6a didn't trigger), but the baselink
  // speed changes slightly because it is recomputed from the limited steering
  // angle via swToBaselink: v.x = sw * cos(limited_sa).
  double pre_limited_sw = std::hypot(0.2, WHEELBASE * pre_limited_tw);
  double expected_vx = velocity_polygon_->callSteeringToBaselinkSpeed(
    pre_limited_sw, expected_boundary_sa);
  EXPECT_NEAR(action.req_vel.x, expected_vx, 1e-6);
}

TEST_F(Tester, testValidateSteeringStep6bUsesSteeringWheelSpeedNotBaselinkX)
{
  // Regression test: the old code compared baselink-x against
  // valid_limit_sw * cos(neighbour_angle), which made the threshold lower than
  // valid_limit_sw. This caused false-positive steering clamping when the robot's
  // steering wheel speed was actually within the neighbour's valid field max.
  //
  // Setup: straight has slow[0,1.0] sa[-0.5,0.5]
  //        left has slow[0,0.5] sa[0.5,1.0]
  // Robot at sa=0, odom x=0.45, tw=0 → sw_speed=0.45.
  // valid_limit_sw = 0.5 (left_slow max). 0.45 < 0.5 → robot CAN enter left.
  // OLD code: 0.45 > 0.5*cos(0.5)=0.439 → wrongly clamped steering.
  // NEW code: 0.45 > 0.5 → false → correctly allows steering.
  setSteeringVelocityPolygonParameters(WHEELBASE, LOW_SPEED_THRESHOLD,
    {"straight_slow", "left_slow"});
  addSteeringAngleSubPolygon("straight_slow", 0.0, 1.0, -0.5, 0.5, STEERING_POLYGON_SLOW_STR);
  addSteeringAngleSubPolygon("left_slow", 0.0, 0.5, 0.5, 1.0, STEERING_POLYGON_SLOW_STR);
  createSteeringVelocityPolygon("limit");

  nav2_collision_monitor::Velocity vel{0.45, 0.0, 0.0};
  velocity_polygon_->updatePolygon(vel);

  // Current: straight at 0.45 (sw_speed=0.45, sa=0)
  nav2_collision_monitor::Velocity odom_vel{0.45, 0.0, 0.0};
  // Target: left
  double target_sa = 0.7;
  double target_tw = std::tan(target_sa) * 0.45 / WHEELBASE;
  nav2_collision_monitor::Velocity cmd_vel{0.45, 0.0, target_tw};

  std::unordered_map<std::string, std::vector<nav2_collision_monitor::Point>> collision_map;
  collision_map["source"] = {};  // no collisions

  // Pre-limited action velocity below the effective limit to avoid 6a triggering.
  // effective_limit = min(current_bucket_limit=1.0, valid_limit=0.5) = 0.5
  // effective_max_baselink = 0.5 * cos(0.5) ≈ 0.439
  // Set result to 0.3 (well below 0.439) so 6a does NOT fire.
  double action_tw = std::tan(target_sa) * 0.3 / WHEELBASE;
  nav2_collision_monitor::Velocity action_vel{0.3, 0.0, action_tw};
  nav2_collision_monitor::Action action{
    nav2_collision_monitor::DO_NOTHING, action_vel, ""};

  bool modified = velocity_polygon_->validateSteering(cmd_vel, odom_vel, collision_map, action);
  // The robot's sw_speed (0.45) is within left_slow's max (0.5), so 6b should NOT
  // trigger. The old baselink-x comparison would have incorrectly clamped steering.
  EXPECT_FALSE(modified);
}

// ============ Step 6: advance angle only as far as the current speed allows ============
//
// A "staircase" of buckets whose max speed drops as the steering angle grows:
//   straight [-0.1, 0.1]  : slow[0,0.5] + mid[0.5,1.0] + fast[1.0,1.5]  (max 1.5)
//   b1       [0.1, 0.52]  : slow[0,0.5] + mid[0.5,1.0]                  (max 1.0)
//   b2       [0.52, 0.87] : slow[0,0.5]                                (max 0.5)
//   b3       [0.87, 1.571]: slow[0,0.3]                                (max 0.3)  ← 90°
// The target is always ~90° (b3) at 0.25 m/s; only the current speed changes.
void Tester::addSteeringStaircase()
{
  addSteeringAngleSubPolygon("straight_s", 0.0, 0.5, -0.1, 0.1, STEERING_POLYGON_SLOW_STR);
  addSteeringAngleSubPolygon("straight_m", 0.5, 1.0, -0.1, 0.1, STEERING_POLYGON_SLOW_STR);
  addSteeringAngleSubPolygon("straight_f", 1.0, 1.5, -0.1, 0.1, STEERING_POLYGON_SLOW_STR);
  addSteeringAngleSubPolygon("b1_s", 0.0, 0.5, 0.1, 0.52, STEERING_POLYGON_SLOW_STR);
  addSteeringAngleSubPolygon("b1_m", 0.5, 1.0, 0.1, 0.52, STEERING_POLYGON_SLOW_STR);
  addSteeringAngleSubPolygon("b2_s", 0.0, 0.5, 0.52, 0.87, STEERING_POLYGON_SLOW_STR);
  addSteeringAngleSubPolygon("b3_s", 0.0, 0.3, 0.87, 1.571, STEERING_POLYGON_SLOW_STR);
}

TEST_F(Tester, testStep6CliffFromFastStraightDoesNotSendTargetAngle)
{
  // (1) From 1.5 m/s straight toward 90° @ 0.25 m/s: at 1.5 m/s the very next
  // bucket (b1, max 1.0) is already a cliff, so the wheel must NOT be commanded
  // toward 90° — it holds near the straight-bucket boundary while slowing.
  setSteeringVelocityPolygonParameters(WHEELBASE, LOW_SPEED_THRESHOLD,
    {"straight_s", "straight_m", "straight_f", "b1_s", "b1_m", "b2_s", "b3_s"});
  addSteeringStaircase();
  createSteeringVelocityPolygon("limit");

  const double target_sw = 0.25, target_angle = 1.5;  // ~90° right-hand bucket (b3)
  nav2_collision_monitor::Velocity cmd_vel{
    target_sw * std::cos(target_angle), 0.0, target_sw * std::sin(target_angle) / WHEELBASE};
  nav2_collision_monitor::Velocity odom_vel{1.5, 0.0, 0.0};  // straight, fast
  velocity_polygon_->updatePolygon(odom_vel);

  std::unordered_map<std::string, std::vector<nav2_collision_monitor::Point>> collision_map;
  collision_map["source"] = {};

  nav2_collision_monitor::Action action{nav2_collision_monitor::DO_NOTHING, cmd_vel, ""};
  bool modified = velocity_polygon_->validateSteering(cmd_vel, odom_vel, collision_map, action);
  EXPECT_TRUE(modified);
  double result_sa = velocity_polygon_->callComputeSteeringAngle(action.req_vel);
  // Held at the straight boundary (0.1 - angle_margin), nowhere near 90°.
  EXPECT_LT(result_sa, 0.2);
  EXPECT_NEAR(result_sa, 0.1 - 0.01, 0.03);
}

TEST_F(Tester, testStep6SlowStraightReachesTargetAngle)
{
  // (2) From 0.25 m/s straight toward 90° @ 0.25 m/s: every bucket up to 90°
  // admits 0.25 m/s, so the target angle is reachable directly — send 90°.
  setSteeringVelocityPolygonParameters(WHEELBASE, LOW_SPEED_THRESHOLD,
    {"straight_s", "straight_m", "straight_f", "b1_s", "b1_m", "b2_s", "b3_s"});
  addSteeringStaircase();
  createSteeringVelocityPolygon("limit");

  const double target_sw = 0.25, target_angle = 1.5;
  nav2_collision_monitor::Velocity cmd_vel{
    target_sw * std::cos(target_angle), 0.0, target_sw * std::sin(target_angle) / WHEELBASE};
  nav2_collision_monitor::Velocity odom_vel{0.25, 0.0, 0.0};  // straight, slow
  velocity_polygon_->updatePolygon(odom_vel);

  std::unordered_map<std::string, std::vector<nav2_collision_monitor::Point>> collision_map;
  collision_map["source"] = {};

  nav2_collision_monitor::Action action{nav2_collision_monitor::DO_NOTHING, cmd_vel, ""};
  velocity_polygon_->validateSteering(cmd_vel, odom_vel, collision_map, action);
  double result_sa = velocity_polygon_->callComputeSteeringAngle(action.req_vel);
  // Full target angle is commanded.
  EXPECT_NEAR(result_sa, target_angle, 0.05);
}

TEST_F(Tester, testStep6MidStraightAdvancesToIntermediateBucket)
{
  // (3) From 1.0 m/s straight toward 90° @ 0.25 m/s: b1 (max 1.0) admits 1.0 but
  // b2 (max 0.5) does not, so the wheel advances to the far edge of b1 (~0.52 rad
  // ≈ 30°) and no further, while slowing toward b2's max.
  setSteeringVelocityPolygonParameters(WHEELBASE, LOW_SPEED_THRESHOLD,
    {"straight_s", "straight_m", "straight_f", "b1_s", "b1_m", "b2_s", "b3_s"});
  addSteeringStaircase();
  createSteeringVelocityPolygon("limit");

  const double target_sw = 0.25, target_angle = 1.5;
  nav2_collision_monitor::Velocity cmd_vel{
    target_sw * std::cos(target_angle), 0.0, target_sw * std::sin(target_angle) / WHEELBASE};
  nav2_collision_monitor::Velocity odom_vel{1.0, 0.0, 0.0};  // straight, mid
  velocity_polygon_->updatePolygon(odom_vel);

  std::unordered_map<std::string, std::vector<nav2_collision_monitor::Point>> collision_map;
  collision_map["source"] = {};

  nav2_collision_monitor::Action action{nav2_collision_monitor::DO_NOTHING, cmd_vel, ""};
  bool modified = velocity_polygon_->validateSteering(cmd_vel, odom_vel, collision_map, action);
  EXPECT_TRUE(modified);
  double result_sa = velocity_polygon_->callComputeSteeringAngle(action.req_vel);
  // Advanced into b1 but stopped at its far edge (0.52 - angle_margin ≈ 0.51),
  // i.e. did not enter b2 and did not reach the 90° target.
  EXPECT_GT(result_sa, 0.1);
  EXPECT_LT(result_sa, 0.87);
  EXPECT_NEAR(result_sa, 0.52 - 0.01, 0.05);
}

TEST_F(Tester, testStep6MultiBucketAdvanceCapsSpeedToReachableBucket)
{
  // Advancing through a reachable bucket to a hard coverage boundary must cap the
  // commanded speed to the reachable bucket's max, not leave it at the (higher)
  // current-bucket limit. straight supports up to 1.0; b1 only up to 0.5; there
  // is no bucket beyond b1. Target is past b1 at a high speed (0.9) b1 cannot hold.
  setSteeringVelocityPolygonParameters(WHEELBASE, LOW_SPEED_THRESHOLD,
    {"straight_s", "straight_m", "b1_s"});
  addSteeringAngleSubPolygon("straight_s", 0.0, 0.5, -0.1, 0.1, STEERING_POLYGON_SLOW_STR);
  addSteeringAngleSubPolygon("straight_m", 0.5, 1.0, -0.1, 0.1, STEERING_POLYGON_FAST_STR);
  addSteeringAngleSubPolygon("b1_s", 0.0, 0.5, 0.1, 0.5, STEERING_POLYGON_SLOW_STR);
  createSteeringVelocityPolygon("limit");

  nav2_collision_monitor::Velocity odom_vel{0.3, 0.0, 0.0};  // straight, fits b1's [0,0.5]
  velocity_polygon_->updatePolygon(odom_vel);
  const double target_sw = 0.9, target_angle = 0.6;  // past b1 (max 0.5 rad), high speed
  nav2_collision_monitor::Velocity cmd_vel{
    target_sw * std::cos(target_angle), 0.0, target_sw * std::sin(target_angle) / WHEELBASE};

  std::unordered_map<std::string, std::vector<nav2_collision_monitor::Point>> collision_map;
  collision_map["source"] = {};

  nav2_collision_monitor::Action action{nav2_collision_monitor::DO_NOTHING, cmd_vel, ""};
  bool modified = velocity_polygon_->validateSteering(cmd_vel, odom_vel, collision_map, action);
  EXPECT_TRUE(modified);
  // Commanded steering-wheel speed must not exceed b1's max (0.5).
  double result_sw = velocity_polygon_->callBaselinkToSteeringSpeed(
    action.req_vel.x, action.req_vel.tw);
  EXPECT_LE(std::abs(result_sw), 0.5 + 1e-6);
}

TEST_F(Tester, testStep6BackwardSlowReachesTargetAngle)
{
  // Reversing slowly toward a fully-turned angle: every backward bucket admits the
  // current speed, so the target angle is reachable directly. Regression for the
  // sign bug where backward fields ([-0.5, 0], linear_max_ ~ 0) were never
  // recognized as reachable and the wheel froze near the straight bucket.
  setSteeringVelocityPolygonParameters(WHEELBASE, LOW_SPEED_THRESHOLD,
    {"bw_straight_s", "bw_straight_m", "bw_b1", "bw_b2", "bw_b3"});
  addSteeringAngleSubPolygon("bw_straight_s", -0.5, 0.0, -0.1, 0.1, STEERING_POLYGON_SLOW_STR);
  addSteeringAngleSubPolygon("bw_straight_m", -1.0, -0.5, -0.1, 0.1, STEERING_POLYGON_FAST_STR);
  addSteeringAngleSubPolygon("bw_b1", -0.5, 0.0, 0.1, 0.52, STEERING_POLYGON_SLOW_STR);
  addSteeringAngleSubPolygon("bw_b2", -0.5, 0.0, 0.52, 0.87, STEERING_POLYGON_SLOW_STR);
  addSteeringAngleSubPolygon("bw_b3", -0.3, 0.0, 0.87, 1.571, STEERING_POLYGON_SLOW_STR);
  createSteeringVelocityPolygon("limit");

  const double sw = 0.25, target_angle = 1.5;  // ~86° left, reversing
  nav2_collision_monitor::Velocity cmd_vel{
    -sw * std::cos(target_angle), 0.0, -sw * std::sin(target_angle) / WHEELBASE};
  nav2_collision_monitor::Velocity odom_vel{-0.25, 0.0, 0.0};  // straight, reversing slow
  velocity_polygon_->updatePolygon(odom_vel);

  std::unordered_map<std::string, std::vector<nav2_collision_monitor::Point>> collision_map;
  collision_map["source"] = {};

  nav2_collision_monitor::Action action{nav2_collision_monitor::DO_NOTHING, cmd_vel, ""};
  velocity_polygon_->validateSteering(cmd_vel, odom_vel, collision_map, action);
  double result_sa = velocity_polygon_->callComputeSteeringAngle(action.req_vel);
  // Reaches the reversing target angle instead of freezing near straight.
  EXPECT_NEAR(result_sa, target_angle, 0.05);
  EXPECT_LT(action.req_vel.x, 0.0);  // still reversing
}

TEST_F(Tester, testFieldsModeDefaultFiltering)
{
  createVelocityPolygonWithModes("stop");
  ASSERT_EQ(velocity_polygon_->getFieldsMode(), "default");

  nav2_collision_monitor::Velocity vel{0.3, 0.0, 0.0};
  velocity_polygon_->updatePolygon(vel);
  EXPECT_EQ(velocity_polygon_->getCurrentSubPolygonName(), "ForwardDefault");

  vel = {0.8, 0.0, 0.0};
  velocity_polygon_->updatePolygon(vel);
  EXPECT_EQ(velocity_polygon_->getCurrentSubPolygonName(), "ForwardDefault");

  vel = {-0.3, 0.0, 0.0};
  velocity_polygon_->updatePolygon(vel);
  EXPECT_EQ(velocity_polygon_->getCurrentSubPolygonName(), "Backward");
}

TEST_F(Tester, testFieldsModeForkDownFiltering)
{
  createVelocityPolygonWithModes("stop");
  velocity_polygon_->setFieldsMode("fork_down");
  ASSERT_EQ(velocity_polygon_->getFieldsMode(), "fork_down");

  nav2_collision_monitor::Velocity vel{0.3, 0.0, 0.0};
  velocity_polygon_->updatePolygon(vel);
  EXPECT_EQ(velocity_polygon_->getCurrentSubPolygonName(), "ForwardForkDown");

  // 0.8 exceeds ForwardForkDown max (0.5), ForwardDefault not in fork_down mode
  vel = {0.8, 0.0, 0.0};
  velocity_polygon_->updatePolygon(vel);
  EXPECT_EQ(velocity_polygon_->getCurrentSubPolygonName(), "none");

  vel = {-0.3, 0.0, 0.0};
  velocity_polygon_->updatePolygon(vel);
  EXPECT_EQ(velocity_polygon_->getCurrentSubPolygonName(), "Backward");
}

TEST_F(Tester, testFieldsModeUnknownModeFiltersAll)
{
  createVelocityPolygonWithModes("stop");
  velocity_polygon_->setFieldsMode("nonexistent_mode");

  nav2_collision_monitor::Velocity vel{0.3, 0.0, 0.0};
  velocity_polygon_->updatePolygon(vel);
  EXPECT_EQ(velocity_polygon_->getCurrentSubPolygonName(), "none");
}

TEST_F(Tester, testFieldsModeSwitching)
{
  createVelocityPolygonWithModes("stop");
  nav2_collision_monitor::Velocity vel{0.3, 0.0, 0.0};

  velocity_polygon_->updatePolygon(vel);
  EXPECT_EQ(velocity_polygon_->getCurrentSubPolygonName(), "ForwardDefault");

  velocity_polygon_->setFieldsMode("fork_down");
  velocity_polygon_->updatePolygon(vel);
  EXPECT_EQ(velocity_polygon_->getCurrentSubPolygonName(), "ForwardForkDown");

  velocity_polygon_->setFieldsMode("default");
  velocity_polygon_->updatePolygon(vel);
  EXPECT_EQ(velocity_polygon_->getCurrentSubPolygonName(), "ForwardDefault");
}

TEST_F(Tester, testFieldsModeSubPolygonModesStored)
{
  createVelocityPolygonWithModes("stop");
  auto sub_polygons = velocity_polygon_->getSubPolygons();
  ASSERT_EQ(sub_polygons.size(), 3u);

  ASSERT_EQ(sub_polygons[0].modes_.size(), 1u);
  EXPECT_EQ(sub_polygons[0].modes_[0], "default");

  ASSERT_EQ(sub_polygons[1].modes_.size(), 1u);
  EXPECT_EQ(sub_polygons[1].modes_[0], "fork_down");

  ASSERT_EQ(sub_polygons[2].modes_.size(), 2u);
  EXPECT_EQ(sub_polygons[2].modes_[0], "default");
  EXPECT_EQ(sub_polygons[2].modes_[1], "fork_down");
}

TEST_F(Tester, testNoModesParameterDefaultsToAlwaysActive)
{
  createVelocityPolygon("stop", IS_NOT_HOLONOMIC);
  auto sub_polygons = velocity_polygon_->getSubPolygons();
  ASSERT_EQ(sub_polygons.size(), 2u);

  ASSERT_EQ(sub_polygons[0].modes_.size(), 1u);
  EXPECT_EQ(sub_polygons[0].modes_[0], "default");

  nav2_collision_monitor::Velocity vel{0.3, 0.0, 0.0};
  velocity_polygon_->updatePolygon(vel);
  EXPECT_EQ(velocity_polygon_->getCurrentSubPolygonName(), "Forward");
}

// ============ Fields mode filtering on steering (lidar e-stop prevention) ============

// findField and findFieldsForAngle must ignore sub-polygons whose `modes` list
// does not contain the current fields mode — even if speed/angle overlap.
TEST_F(Tester, testFieldsModeFiltersSteeringFieldLookup)
{
  setSteeringVelocityPolygonParameters(WHEELBASE, LOW_SPEED_THRESHOLD,
    {"right_default_slow", "right_default_fast", "right_narrow_slow"});
  // Default mode: right allows slow [0, 0.5] + fast [0.5, 1.0]
  addSteeringAngleSubPolygon(
    "right_default_slow", 0.0, 0.5, -0.5, -0.1, STEERING_POLYGON_SLOW_STR, {"default"});
  addSteeringAngleSubPolygon(
    "right_default_fast", 0.5, 1.0, -0.5, -0.1, STEERING_POLYGON_FAST_STR, {"default"});
  // Narrow mode: right allows only a slow field [0, 0.3] covering the same angle bucket
  addSteeringAngleSubPolygon(
    "right_narrow_slow", 0.0, 0.3, -0.5, -0.1, STEERING_POLYGON_SLOW_STR,
    {"narrow_fork_down"});
  createSteeringVelocityPolygon("limit");

  // --- findField: only the default sub-polygons are visible in default mode ---
  auto field = velocity_polygon_->callFindField(0.2, -0.3);
  ASSERT_NE(field, nullptr);
  EXPECT_EQ(field->velocity_polygon_name_, "right_default_slow");
  field = velocity_polygon_->callFindField(0.7, -0.3);
  ASSERT_NE(field, nullptr);
  EXPECT_EQ(field->velocity_polygon_name_, "right_default_fast");

  // --- findFieldsForAngle: both default fields returned, sorted slowest first ---
  auto fields = velocity_polygon_->callFindFieldsForAngle(-0.3, true);
  ASSERT_EQ(fields.size(), 2u);
  EXPECT_EQ(fields[0]->velocity_polygon_name_, "right_default_slow");
  EXPECT_EQ(fields[1]->velocity_polygon_name_, "right_default_fast");

  // --- Switch to narrow_fork_down: only narrow field visible ---
  velocity_polygon_->setFieldsMode("narrow_fork_down");

  // 0.2 still matches narrow slow field
  field = velocity_polygon_->callFindField(0.2, -0.3);
  ASSERT_NE(field, nullptr);
  EXPECT_EQ(field->velocity_polygon_name_, "right_narrow_slow");
  // 0.7 exceeds narrow's max (0.3) and the default fast field must NOT be picked up
  field = velocity_polygon_->callFindField(0.7, -0.3);
  EXPECT_EQ(field, nullptr);

  fields = velocity_polygon_->callFindFieldsForAngle(-0.3, true);
  ASSERT_EQ(fields.size(), 1u);
  EXPECT_EQ(fields[0]->velocity_polygon_name_, "right_narrow_slow");

  // --- Unknown mode: no fields match ---
  velocity_polygon_->setFieldsMode("fork_down");
  EXPECT_EQ(velocity_polygon_->callFindField(0.2, -0.3), nullptr);
  EXPECT_EQ(velocity_polygon_->callFindFieldsForAngle(-0.3, true).size(), 0u);
}

// Headline case: when in narrow_fork_down mode, turning right is only allowed
// slowly. clampToMaxField must clamp the commanded speed down to the narrow
// field's max rather than picking up the default (higher) max that overlaps
// the same angle bucket in default mode.
TEST_F(Tester, testFieldsModeNarrowRightSpeedLimitClamped)
{
  setSteeringVelocityPolygonParameters(WHEELBASE, LOW_SPEED_THRESHOLD,
    {"right_default_slow", "right_default_fast", "right_narrow_slow"});
  addSteeringAngleSubPolygon(
    "right_default_slow", 0.0, 0.5, -0.5, -0.1, STEERING_POLYGON_SLOW_STR, {"default"});
  addSteeringAngleSubPolygon(
    "right_default_fast", 0.5, 1.0, -0.5, -0.1, STEERING_POLYGON_FAST_STR, {"default"});
  addSteeringAngleSubPolygon(
    "right_narrow_slow", 0.0, 0.3, -0.5, -0.1, STEERING_POLYGON_SLOW_STR,
    {"narrow_fork_down"});
  createSteeringVelocityPolygon("limit");

  // Robot physically turned right at sa ≈ -0.3, low baselink speed
  const double phys_sa = -0.3;
  const double phys_speed = 0.1;
  const double phys_tw = std::tan(phys_sa) * phys_speed / WHEELBASE;
  nav2_collision_monitor::Velocity odom_vel{phys_speed, 0.0, phys_tw};

  // Command 0.7 m/s at the same right-turn angle
  const double cmd_speed = 0.7;
  const double cmd_tw = std::tan(phys_sa) * cmd_speed / WHEELBASE;
  nav2_collision_monitor::Velocity cmd_vel{cmd_speed, 0.0, cmd_tw};

  // Default mode: 0.7 m/s is within right_default_fast (max 1.0) → no clamping.
  {
    nav2_collision_monitor::Action action{
      nav2_collision_monitor::DO_NOTHING, cmd_vel, ""};
    bool modified = velocity_polygon_->callClampToMaxField(odom_vel, action);
    EXPECT_FALSE(modified);
  }

  // Narrow mode: only right_narrow_slow (max 0.3) is active at this angle.
  // The commanded speed must be clamped to narrow's max, decomposed via cos(sa)
  // and inset by the speed margin — NOT to default's 1.0.
  {
    velocity_polygon_->setFieldsMode("narrow_fork_down");
    nav2_collision_monitor::Action action{
      nav2_collision_monitor::DO_NOTHING, cmd_vel, ""};
    bool modified = velocity_polygon_->callClampToMaxField(odom_vel, action);
    EXPECT_TRUE(modified);
    EXPECT_EQ(action.action_type, nav2_collision_monitor::LIMIT);
    // Upper bound: narrow max (0.3) * cos(phys_sa), plus tolerance for margin.
    const double expected_upper_baselink = 0.3 * std::cos(phys_sa);
    EXPECT_LE(std::abs(action.req_vel.x), expected_upper_baselink + 1e-3);
    // Must actually be reduced below default's 1.0 * cos(phys_sa).
    EXPECT_LT(std::abs(action.req_vel.x), 1.0 * std::cos(phys_sa));
    // Clamped steering wheel speed lands just below the narrow max (0.3).
    const double result_sw = velocity_polygon_->callBaselinkToSteeringSpeed(
      action.req_vel.x, action.req_vel.tw);
    EXPECT_LE(std::abs(result_sw), 0.3 + 1e-6);
  }
}

// ==================== limiter invariant on the baselink speed ====================

// Regression for the miele-amr1 2026-08-31 side-approach overshoot: a
// pure-rotation command (PrecisionSpin) issued while the robot is still
// rolling backward must never come out of validateSteering as translation.
// Before the invariant clamp, the different-bucket path re-anchored the
// bucket barrier speed at the reachable steering angle, so a (0, 0, wz)
// request from a moving robot was answered with ~0.14 m/s of linear creep
// that drove the robot ~0.4 m off its spin pose.
TEST_F(Tester, testValidateSteeringPureRotationNeverSynthesizesTranslation)
{
  // Backward straight bucket (robot's current state) and a slow forward bucket
  // at right steering angles on the way toward the spin's target angle (-pi/2).
  setSteeringVelocityPolygonParameters(WHEELBASE, LOW_SPEED_THRESHOLD,
    {"bw_straight", "fw_right_slow"});
  addSteeringAngleSubPolygon("bw_straight", -0.5, 0.0, -0.3, 0.3, STEERING_POLYGON_SLOW_STR);
  addSteeringAngleSubPolygon("fw_right_slow", 0.0, 0.15, -1.6, -0.3, STEERING_POLYGON_SLOW_STR);
  createSteeringVelocityPolygon("limit");

  nav2_collision_monitor::Velocity vel{-0.24, 0.0, 0.0};
  velocity_polygon_->updatePolygon(vel);

  // Spin command while still rolling backward (the DriveOnHeading cancel case)
  nav2_collision_monitor::Velocity cmd_vel{0.0, 0.0, -0.38};
  nav2_collision_monitor::Velocity odom_vel{-0.24, 0.0, 0.0};
  std::unordered_map<std::string, std::vector<nav2_collision_monitor::Point>> collision_map;
  // Occupy the intermediate bucket (the rack the robot backed toward)
  collision_map["source"] = {
    {0.0, 0.0},
    {0.2, 0.1},
  };

  nav2_collision_monitor::Action action{
    nav2_collision_monitor::DO_NOTHING, cmd_vel, ""};

  bool modified = velocity_polygon_->validateSteering(cmd_vel, odom_vel, collision_map, action);
  EXPECT_TRUE(modified);
  EXPECT_EQ(action.action_type, nav2_collision_monitor::LIMIT);
  // The planner asked for zero linear speed — the validator must not invent any.
  // The robot then simply decelerates to standstill, after which the
  // standstill/startup paths release the steering toward the spin angle.
  EXPECT_EQ(action.req_vel.x, 0.0);
  // The rotation channel may survive scaled toward zero, but never flipped
  // in sign or amplified beyond the request.
  EXPECT_LE(action.req_vel.tw, 0.0);
  EXPECT_LE(std::abs(action.req_vel.tw), std::abs(cmd_vel.tw) + 1e-9);
}

// The direction-reversal angle hold converts the requested speed through a
// different steering angle than the planner's, which amplified the baselink
// speed above the request (0.5 requested → ~0.57 commanded). The invariant
// caps the speed at the requested magnitude while the twist keeps encoding
// the held (current) steering angle.
TEST_F(Tester, testValidateSteeringReversalNeverAmplifiesSpeed)
{
  setSteeringVelocityPolygonParameters(WHEELBASE, LOW_SPEED_THRESHOLD, {"slow", "fast"});
  addSteeringAngleSubPolygon("slow", 0.0, 0.5, -0.5, 0.5, STEERING_POLYGON_SLOW_STR);
  addSteeringAngleSubPolygon("fast", 0.5, 1.0, -0.5, 0.5, STEERING_POLYGON_FAST_STR);
  createSteeringVelocityPolygon("limit");

  nav2_collision_monitor::Velocity vel{0.3, 0.0, 0.0};
  velocity_polygon_->updatePolygon(vel);

  // Planner wants forward with steering; robot still moves backward.
  nav2_collision_monitor::Velocity cmd_vel{0.5, 0.0, 0.3};
  nav2_collision_monitor::Velocity odom_vel{-0.5, 0.0, 0.1};
  std::unordered_map<std::string, std::vector<nav2_collision_monitor::Point>> collision_map;
  collision_map["source"] = {};

  nav2_collision_monitor::Action action{
    nav2_collision_monitor::DO_NOTHING, cmd_vel, ""};

  bool modified = velocity_polygon_->validateSteering(cmd_vel, odom_vel, collision_map, action);
  EXPECT_TRUE(modified);
  EXPECT_EQ(action.action_type, nav2_collision_monitor::LIMIT);
  // Same direction as requested, never faster than requested.
  EXPECT_GT(action.req_vel.x, 0.0);
  EXPECT_LE(std::abs(action.req_vel.x), std::abs(cmd_vel.x) + 1e-9);
  // The twist still encodes the held (current) steering angle.
  const double held_sa = velocity_polygon_->callComputeSteeringAngle(odom_vel);
  const double result_sa = velocity_polygon_->callComputeSteeringAngle(action.req_vel);
  EXPECT_NEAR(result_sa, held_sa, 1e-6);
}

// Regression for the miele-amr1 2026-08-31 10:38 UTC dead spins: a pure-rotation
// command at standstill must be answered with a capped same-sign yaw rate, not
// zeroed. Before the standstill hemisphere snap, target_sw's sign came from
// cmd.x == 0 ("forward") while current_sw's sign was odometry noise ("backward"),
// so the same-bucket limit ran over backward fields (poisoned by the rack behind
// the forks), the bucket walk ran over forward fields and stalled on the sign bit,
// and the result carried the backward barrier — which the limiter invariant then
// rightly rejected to (0, 0).
TEST_F(Tester, testValidateSteeringStandstillSpinCappedNotZeroed)
{
  setSteeringVelocityPolygonParameters(WHEELBASE, LOW_SPEED_THRESHOLD,
    {"fw_straight_slow", "bw_straight_slow", "bw_straight_next", "fw_90R_slow"});
  addSteeringAngleSubPolygon("fw_straight_slow", 0.0, 0.15, -0.3, 0.3, STEERING_POLYGON_SLOW_STR);
  // The robot's straight backward bucket, and its next-faster field occupied by
  // structure behind the forks (points inside the FAST polygon only)
  addSteeringAngleSubPolygon("bw_straight_slow", -0.15, 0.0, -0.3, 0.3, STEERING_POLYGON_SLOW_STR);
  addSteeringAngleSubPolygon("bw_straight_next", -0.5, -0.3, -0.3, 0.3, STEERING_POLYGON_FAST_STR);
  // Free slow field covering the spin's target angle (-pi/2) in the snapped
  // (target) hemisphere
  addSteeringAngleSubPolygon("fw_90R_slow", 0.0, 0.15, -1.5708, -0.3, STEERING_POLYGON_SLOW_STR);
  createSteeringVelocityPolygon("limit");

  nav2_collision_monitor::Velocity vel{0.0, 0.0, 0.0};
  velocity_polygon_->updatePolygon(vel);

  // Spin command at standstill; odom carries a negative noise sign
  nav2_collision_monitor::Velocity cmd_vel{0.0, 0.0, -0.35};
  nav2_collision_monitor::Velocity odom_vel{-1e-9, 0.0, -1e-12};
  std::unordered_map<std::string, std::vector<nav2_collision_monitor::Point>> collision_map;
  // Inside the FAST polygon but outside the SLOW one → only bw_straight_next occupied
  collision_map["source"] = {
    {1.2, 0.5}, {1.3, 0.6}, {1.2, 0.55}, {1.25, 0.5},
  };

  nav2_collision_monitor::Action action{
    nav2_collision_monitor::DO_NOTHING, cmd_vel, ""};

  bool modified = velocity_polygon_->validateSteering(cmd_vel, odom_vel, collision_map, action);
  EXPECT_TRUE(modified);
  EXPECT_EQ(action.action_type, nav2_collision_monitor::LIMIT);
  // No synthesized translation
  EXPECT_EQ(action.req_vel.x, 0.0);
  // The spin survives: same sign as requested, capped by the target-angle slow
  // field's bound (0.15 minus margin, over wheelbase 1.0) — nonzero.
  EXPECT_LT(action.req_vel.tw, -0.05);
  EXPECT_GE(action.req_vel.tw, -0.35);
}

// With no field covering the target angle, the walk holds at the last covered
// bucket edge: the spin comes out heavily limited (steering only up to the edge)
// but never with a flipped sign and never as synthesized translation.
TEST_F(Tester, testValidateSteeringStandstillSpinHoldsAtBucketEdgeWithoutTargetField)
{
  setSteeringVelocityPolygonParameters(WHEELBASE, LOW_SPEED_THRESHOLD,
    {"bw_straight_slow", "fw_straight_slow"});
  addSteeringAngleSubPolygon("bw_straight_slow", -0.15, 0.0, -0.3, 0.3, STEERING_POLYGON_SLOW_STR);
  addSteeringAngleSubPolygon("fw_straight_slow", 0.0, 0.15, -0.3, 0.3, STEERING_POLYGON_SLOW_STR);
  createSteeringVelocityPolygon("limit");

  nav2_collision_monitor::Velocity vel{0.0, 0.0, 0.0};
  velocity_polygon_->updatePolygon(vel);

  nav2_collision_monitor::Velocity cmd_vel{0.0, 0.0, -0.35};
  nav2_collision_monitor::Velocity odom_vel{-1e-9, 0.0, 0.0};
  std::unordered_map<std::string, std::vector<nav2_collision_monitor::Point>> collision_map;
  collision_map["source"] = {};

  nav2_collision_monitor::Action action{
    nav2_collision_monitor::DO_NOTHING, cmd_vel, ""};

  bool modified = velocity_polygon_->validateSteering(cmd_vel, odom_vel, collision_map, action);
  EXPECT_TRUE(modified);
  EXPECT_EQ(action.req_vel.x, 0.0);
  EXPECT_LE(action.req_vel.tw, 0.0);
  EXPECT_GE(action.req_vel.tw, -0.35);
}

int main(int argc, char ** argv)
{
  // Initialize the system
  testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);

  // Actual testing
  bool test_result = RUN_ALL_TESTS();

  // Shutdown
  rclcpp::shutdown();

  return test_result;
}
