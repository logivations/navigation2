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
    const std::string & polygon_points, const bool is_holonomic);

  void addSteeringAngleSubPolygon(
    const std::string & sub_polygon_name,
    const double linear_min, const double linear_max,
    const double steering_angle_min, const double steering_angle_max,
    const std::string & polygon_points);
  void setSteeringVelocityPolygonParameters(
    const double wheelbase, const double low_speed_threshold,
    const std::vector<std::string> & sub_polygon_names);

  // Creating routines
  void createVelocityPolygon(const std::string & action_type, const bool is_holonomic);
  void createSteeringVelocityPolygon(const std::string & action_type);

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
  const std::string & polygon_points, const bool is_holonomic)
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
}

void Tester::addSteeringAngleSubPolygon(
  const std::string & sub_polygon_name,
  const double linear_min, const double linear_max,
  const double steering_angle_min, const double steering_angle_max,
  const std::string & polygon_points)
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
  // Speed (0.3) is already within the slowest field's max (0.5), so nothing to change
  EXPECT_FALSE(modified);
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
