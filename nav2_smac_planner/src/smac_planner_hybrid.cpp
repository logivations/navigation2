// Copyright (c) 2020, Samsung Research America
// Copyright (c) 2023, Open Navigation LLC
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
// limitations under the License. Reserved.

#include <string>
#include <memory>
#include <vector>
#include <algorithm>
#include <limits>
#include <cmath>
#include <queue>
#include <utility>

#include "nav2_smac_planner/smac_planner_hybrid.hpp"

// #define BENCHMARK_TESTING

namespace nav2_smac_planner
{

using namespace std::chrono;  // NOLINT
using rcl_interfaces::msg::ParameterType;
using std::placeholders::_1;

SmacPlannerHybrid::SmacPlannerHybrid()
: _a_star(nullptr),
  _collision_checker(nullptr, 1, nullptr),
  _smoother(nullptr),
  _costmap(nullptr),
  _costmap_ros(nullptr),
  _costmap_downsampler(nullptr)
{
}

SmacPlannerHybrid::~SmacPlannerHybrid()
{
  RCLCPP_INFO(
    _logger, "Destroying plugin %s of type SmacPlannerHybrid",
    _name.c_str());
}

void SmacPlannerHybrid::configure(
  const nav2::LifecycleNode::WeakPtr & parent,
  std::string name, std::shared_ptr<tf2_ros::Buffer>/*tf*/,
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
{
  _node = parent;
  auto node = parent.lock();
  _logger = node->get_logger();
  _clock = node->get_clock();
  _costmap = costmap_ros->getCostmap();
  _costmap_ros = costmap_ros;
  _name = name;
  _global_frame = costmap_ros->getGlobalFrameID();

  RCLCPP_INFO(_logger, "Configuring %s of type SmacPlannerHybrid", name.c_str());

  int angle_quantizations;
  double analytic_expansion_max_length_m;
  bool smooth_path;

  // General planner params
  _downsample_costmap = node->declare_or_get_parameter(name + ".downsample_costmap", false);
  _downsampling_factor = node->declare_or_get_parameter(name + ".downsampling_factor", 1);

  angle_quantizations = node->declare_or_get_parameter(name + ".angle_quantization_bins", 72);
  _angle_bin_size = 2.0 * M_PI / angle_quantizations;
  _angle_quantizations = static_cast<unsigned int>(angle_quantizations);

  _tolerance = static_cast<float>(node->declare_or_get_parameter(name + ".tolerance", 0.25));
  _allow_unknown = node->declare_or_get_parameter(name + ".allow_unknown", true);
  _max_iterations = node->declare_or_get_parameter(name + ".max_iterations", 1000000);
  _max_on_approach_iterations =
    node->declare_or_get_parameter(name + ".max_on_approach_iterations", 1000);
  _terminal_checking_interval =
    node->declare_or_get_parameter(name + ".terminal_checking_interval", 5000);
  smooth_path = node->declare_or_get_parameter(name + ".smooth_path", true);

  _minimum_turning_radius_global_coords =
    node->declare_or_get_parameter(name + ".minimum_turning_radius", 0.4);
  // Optional asymmetric right-turn radius (0.0 = symmetric)
  _minimum_turning_radius_right_global_coords =
    node->declare_or_get_parameter(name + ".minimum_turning_radius_right", 0.0);
  _search_info.allow_primitive_interpolation =
    node->declare_or_get_parameter(name + ".allow_primitive_interpolation", false);
  _search_info.cache_obstacle_heuristic =
    node->declare_or_get_parameter(name + ".cache_obstacle_heuristic", false);
  _search_info.reverse_penalty =
    node->declare_or_get_parameter(name + ".reverse_penalty", 2.0);
  _search_info.change_penalty =
    node->declare_or_get_parameter(name + ".change_penalty", 0.0);
  _search_info.non_straight_penalty =
    node->declare_or_get_parameter(name + ".non_straight_penalty", 1.2);
  _search_info.cost_penalty =
    node->declare_or_get_parameter(name + ".cost_penalty", 2.0);
  _search_info.retrospective_penalty =
    node->declare_or_get_parameter(name + ".retrospective_penalty", 0.015);
  _search_info.analytic_expansion_ratio =
    node->declare_or_get_parameter(name + ".analytic_expansion_ratio", 3.5);
  _search_info.analytic_expansion_max_cost =
    node->declare_or_get_parameter(name + ".analytic_expansion_max_cost", 200.0);
  _search_info.analytic_expansion_max_cost_override =
    node->declare_or_get_parameter(name + ".analytic_expansion_max_cost_override", false);
  _search_info.use_quadratic_cost_penalty =
    node->declare_or_get_parameter(name + ".use_quadratic_cost_penalty", false);
  _search_info.downsample_obstacle_heuristic =
    node->declare_or_get_parameter(name + ".downsample_obstacle_heuristic", true);

  analytic_expansion_max_length_m =
    node->declare_or_get_parameter(name + ".analytic_expansion_max_length", 3.0);
  _search_info.analytic_expansion_max_length =
    analytic_expansion_max_length_m / _costmap->getResolution();

  _max_planning_time = node->declare_or_get_parameter(name + ".max_planning_time", 5.0);
  _lookup_table_size = node->declare_or_get_parameter(name + ".lookup_table_size", 20.0);

  _debug_visualizations = node->declare_or_get_parameter(name + ".debug_visualizations", false);
  // Master switch for the per-request publish_failed_search opt-in. Unlike
  // debug_visualizations this costs nothing on successful plans: the explored area is read
  // from the search graph after the failure
  _publish_failed_search =
    node->declare_or_get_parameter(name + ".publish_failed_search", true);
  // Reachable part of the way for requests with compute_partial_path, see computePartialPlan
  _partial_path_backoff =
    node->declare_or_get_parameter(name + ".partial_path_backoff", 1.5);
  _partial_path_min_end_headings =
    node->declare_or_get_parameter(name + ".partial_path_min_end_headings", 3);
  _partial_path_blocked_cost_factor =
    node->declare_or_get_parameter(name + ".partial_path_blocked_cost_factor", 20.0);
  _partial_path_allow_reversing =
    node->declare_or_get_parameter(name + ".partial_path_allow_reversing", false);

  _motion_model_for_search =
    node->declare_or_get_parameter(name + ".motion_model_for_search", std::string("DUBIN"));

  std::string goal_heading_type =
    node->declare_or_get_parameter(name + ".goal_heading_mode", std::string("DEFAULT"));
  _goal_heading_mode = fromStringToGH(goal_heading_type);

  _coarse_search_resolution =
    node->declare_or_get_parameter(name + ".coarse_search_resolution", 1);

  // In find free space mode the goal pose is ignored: the planner fans out
  // from the start (Dijkstra) and returns a path to the cheapest-to-reach
  // position outside of the no-waiting zone grid
  _find_free_space_mode = node->declare_or_get_parameter(name + ".find_free_space_mode", false);
  if (_find_free_space_mode) {
    _no_waiting_zone.configure(node, name);
    RCLCPP_INFO(
      _logger, "%s is in find free space mode: goal poses will be ignored and the "
      "no-waiting zone is taken from topic '%s'.",
      _name.c_str(), _no_waiting_zone.getTopic().c_str());
  }

  if (_goal_heading_mode == GoalHeadingMode::UNKNOWN) {
    std::string error_msg = "Unable to get GoalHeader type. Given '" + goal_heading_type + "' "
      "Valid options are DEFAULT, BIDIRECTIONAL, ALL_DIRECTION. ";
    throw nav2_core::PlannerException(error_msg);
  }

  _motion_model = fromString(_motion_model_for_search);

  if (_motion_model == MotionModel::UNKNOWN) {
    RCLCPP_WARN(
      _logger,
      "Unable to get MotionModel search type. Given '%s', "
      "valid options are MOORE, VON_NEUMANN, DUBIN, REEDS_SHEPP, STATE_LATTICE.",
      _motion_model_for_search.c_str());
  }

  if (_max_on_approach_iterations <= 0) {
    RCLCPP_WARN(
      _logger, "On approach iteration selected as <= 0, "
      "disabling tolerance and on approach iterations.");
    _max_on_approach_iterations = std::numeric_limits<int>::max();
  }

  if (_max_iterations <= 0) {
    RCLCPP_WARN(
      _logger, "maximum iteration selected as <= 0, "
      "disabling maximum iterations.");
    _max_iterations = std::numeric_limits<int>::max();
  }

  if (_coarse_search_resolution <= 0) {
    RCLCPP_WARN(
      _logger, "coarse iteration resolution selected as <= 0, "
      "disabling coarse iteration resolution search for goal heading"
    );

    _coarse_search_resolution = 1;
  }

  if (_angle_quantizations % _coarse_search_resolution != 0) {
    std::string error_msg = "coarse iteration should be an increment"
      " of the number of angular bins configured";
    throw nav2_core::PlannerException(error_msg);
  }

  if (_minimum_turning_radius_global_coords < _costmap->getResolution() * _downsampling_factor) {
    RCLCPP_WARN(
      _logger, "Min turning radius cannot be less than the search grid cell resolution!");
    _minimum_turning_radius_global_coords = _costmap->getResolution() * _downsampling_factor;
  }

  if (_minimum_turning_radius_right_global_coords > 0.0 &&
    _minimum_turning_radius_right_global_coords <
    _costmap->getResolution() * _downsampling_factor)
  {
    RCLCPP_WARN(
      _logger, "Min right-turn radius cannot be less than the search grid cell resolution! "
      "Disabling asymmetric turning radius.");
    _minimum_turning_radius_right_global_coords = 0.0;
  }

  // convert to grid coordinates
  if (!_downsample_costmap) {
    _downsampling_factor = 1;
  }
  _search_info.minimum_turning_radius =
    _minimum_turning_radius_global_coords / (_costmap->getResolution() * _downsampling_factor);
  _search_info.minimum_turning_radius_right = _minimum_turning_radius_right_global_coords > 0.0 ?
    _minimum_turning_radius_right_global_coords /
    (_costmap->getResolution() * _downsampling_factor) :
    0.0f;
  _lookup_table_dim =
    static_cast<float>(_lookup_table_size) /
    static_cast<float>(_costmap->getResolution() * _downsampling_factor);

  // Make sure its a whole number
  _lookup_table_dim = static_cast<float>(static_cast<int>(_lookup_table_dim));

  // Make sure its an odd number
  if (static_cast<int>(_lookup_table_dim) % 2 == 0) {
    RCLCPP_INFO(
      _logger,
      "Even sized heuristic lookup table size set %f, increasing size by 1 to make odd",
      _lookup_table_dim);
    _lookup_table_dim += 1.0;
  }

  // Initialize collision checker
  _collision_checker = GridCollisionChecker(_costmap_ros, _angle_quantizations, node);
  _collision_checker.setFootprint(
    _costmap_ros->getRobotFootprint(),
    _costmap_ros->getUseRadius(),
    findCircumscribedCost(_costmap_ros));

  // Initialize A* template
  _a_star = std::make_unique<AStarAlgorithm<NodeHybrid>>(_motion_model, _search_info);
  _a_star->initialize(
    _allow_unknown,
    _max_iterations,
    _max_on_approach_iterations,
    _terminal_checking_interval,
    _max_planning_time,
    _lookup_table_dim,
    _angle_quantizations);

  // Initialize path smoother
  // Smoother uses a single curvature constraint, so use the larger of left/right radii
  // to stay conservative (won't tighten paths beyond the looser side's limit).
  SmootherParams params;
  params.get(node, name);
  if (smooth_path) {
    const double smoother_radius = std::max(
      _minimum_turning_radius_global_coords, _minimum_turning_radius_right_global_coords);
    _smoother = std::make_unique<Smoother>(params);
    _smoother->initialize(smoother_radius);
  }

  // Initialize costmap downsampler
  _costmap_downsampler = std::make_unique<CostmapDownsampler>();
  std::string topic_name = "downsampled_costmap";
  _costmap_downsampler->on_configure(
    node, _global_frame, topic_name, _costmap, _downsampling_factor);

  _raw_plan_publisher = node->create_publisher<nav_msgs::msg::Path>("unsmoothed_plan");

  if (_publish_failed_search) {
    // Next to the planner server's own planner_server/failed_* topics
    _failed_explored_area_publisher = node->create_publisher<nav_msgs::msg::OccupancyGrid>(
      "planner_server/failed_explored_area");
    _failed_closest_path_publisher = node->create_publisher<nav_msgs::msg::Path>(
      "planner_server/failed_closest_path");
    _failed_partial_path_publisher = node->create_publisher<nav_msgs::msg::Path>(
      "planner_server/failed_partial_path");
  }

  if (_debug_visualizations) {
    _expansions_publisher = node->create_publisher<geometry_msgs::msg::PoseArray>("expansions");
    _planned_footprints_publisher = node->create_publisher<visualization_msgs::msg::MarkerArray>(
      "planned_footprints");
    _smoothed_footprints_publisher =
      node->create_publisher<visualization_msgs::msg::MarkerArray>(
      "smoothed_footprints");
  }

  RCLCPP_INFO(
    _logger, "Configured plugin %s of type SmacPlannerHybrid with "
    "maximum iterations %i, max on approach iterations %i, and %s. Tolerance %.2f."
    "Using motion model: %s.",
    _name.c_str(), _max_iterations, _max_on_approach_iterations,
    _allow_unknown ? "allowing unknown traversal" : "not allowing unknown traversal",
    _tolerance, toString(_motion_model).c_str());
}

void SmacPlannerHybrid::activate()
{
  RCLCPP_INFO(
    _logger, "Activating plugin %s of type SmacPlannerHybrid",
    _name.c_str());
  _raw_plan_publisher->on_activate();
  if (_publish_failed_search) {
    _failed_explored_area_publisher->on_activate();
    _failed_closest_path_publisher->on_activate();
    _failed_partial_path_publisher->on_activate();
  }
  if (_debug_visualizations) {
    _expansions_publisher->on_activate();
    _planned_footprints_publisher->on_activate();
    _smoothed_footprints_publisher->on_activate();
  }
  if (_costmap_downsampler) {
    _costmap_downsampler->on_activate();
  }
  auto node = _node.lock();
  // Add callback for dynamic parameters
  _post_set_params_handler = node->add_post_set_parameters_callback(
    std::bind(
      &SmacPlannerHybrid::updateParametersCallback,
      this, std::placeholders::_1));
  _on_set_params_handler = node->add_on_set_parameters_callback(
    std::bind(
      &SmacPlannerHybrid::validateParameterUpdatesCallback,
      this, std::placeholders::_1));

  // Special case handling to obtain resolution changes in global costmap
  auto resolution_remote_cb = [this](const rclcpp::Parameter & p) {
      updateParametersCallback(
        {rclcpp::Parameter("resolution", rclcpp::ParameterValue(p.as_double()))});
    };
  _remote_param_subscriber = std::make_shared<rclcpp::ParameterEventHandler>(_node.lock());
  _remote_resolution_handler = _remote_param_subscriber->add_parameter_callback(
    "resolution", resolution_remote_cb, "global_costmap/global_costmap");
}

void SmacPlannerHybrid::deactivate()
{
  RCLCPP_INFO(
    _logger, "Deactivating plugin %s of type SmacPlannerHybrid",
    _name.c_str());
  _raw_plan_publisher->on_deactivate();
  if (_publish_failed_search) {
    _failed_explored_area_publisher->on_deactivate();
    _failed_closest_path_publisher->on_deactivate();
    _failed_partial_path_publisher->on_deactivate();
  }
  if (_debug_visualizations) {
    _expansions_publisher->on_deactivate();
    _planned_footprints_publisher->on_deactivate();
    _smoothed_footprints_publisher->on_deactivate();
  }
  if (_costmap_downsampler) {
    _costmap_downsampler->on_deactivate();
  }
  // shutdown dyn_param_handler
  auto node = _node.lock();
  if (_post_set_params_handler && node) {
    node->remove_post_set_parameters_callback(_post_set_params_handler.get());
  }
  _post_set_params_handler.reset();
  if (_on_set_params_handler && node) {
    node->remove_on_set_parameters_callback(_on_set_params_handler.get());
  }
  _on_set_params_handler.reset();
}

void SmacPlannerHybrid::cleanup()
{
  RCLCPP_INFO(
    _logger, "Cleaning up plugin %s of type SmacPlannerHybrid",
    _name.c_str());
  _a_star.reset();
  _smoother.reset();
  if (_costmap_downsampler) {
    _costmap_downsampler->on_cleanup();
    _costmap_downsampler.reset();
  }
  _no_waiting_zone.cleanup();
  _raw_plan_publisher.reset();
  _failed_explored_area_publisher.reset();
  _failed_closest_path_publisher.reset();
  _failed_partial_path_publisher.reset();
  if (_debug_visualizations) {
    _expansions_publisher.reset();
    _planned_footprints_publisher.reset();
    _smoothed_footprints_publisher.reset();
  }
}

nav_msgs::msg::Path SmacPlannerHybrid::createPlan(
  const geometry_msgs::msg::PoseStamped & start,
  const geometry_msgs::msg::PoseStamped & goal,
  std::function<bool()> cancel_checker)
{
  return createPlan(start, goal, cancel_checker, nav2_core::PlanRequestOptions());
}

nav_msgs::msg::Path SmacPlannerHybrid::createPlan(
  const geometry_msgs::msg::PoseStamped & start,
  const geometry_msgs::msg::PoseStamped & goal,
  std::function<bool()> cancel_checker,
  const nav2_core::PlanRequestOptions & options)
{
  std::lock_guard<std::mutex> lock_reinit(_mutex);
  steady_clock::time_point a = steady_clock::now();

  std::unique_lock<nav2_costmap_2d::Costmap2D::mutex_t> lock(*(_costmap->getMutex()));

  // Downsample costmap, if required
  nav2_costmap_2d::Costmap2D * costmap = _costmap;
  if (_downsample_costmap && _downsampling_factor > 1) {
    costmap = _costmap_downsampler->downsample(_downsampling_factor);
    _collision_checker.setCostmap(costmap);
  }

  // Set collision checker and costmap information
  _collision_checker.setFootprint(
    _costmap_ros->getRobotFootprint(),
    _costmap_ros->getUseRadius(),
    findCircumscribedCost(_costmap_ros));
  _a_star->setCollisionChecker(&_collision_checker);

  // Set starting point, in A* bin search coordinates
  float mx_start, my_start, mx_goal = 0.0, my_goal = 0.0;
  if (!costmap->worldToMapContinuous(
      start.pose.position.x,
      start.pose.position.y,
      mx_start,
      my_start))
  {
    throw nav2_core::StartOutsideMapBounds(
            "Start Coordinates of(" + std::to_string(start.pose.position.x) + ", " +
            std::to_string(start.pose.position.y) + ") was outside bounds");
  }

  double start_orientation_bin = std::round(tf2::getYaw(start.pose.orientation) / _angle_bin_size);
  while (start_orientation_bin < 0.0) {
    start_orientation_bin += static_cast<float>(_angle_quantizations);
  }
  // This is needed to handle precision issues
  if (start_orientation_bin >= static_cast<float>(_angle_quantizations)) {
    start_orientation_bin -= static_cast<float>(_angle_quantizations);
  }
  unsigned int start_orientation_bin_int =
    static_cast<unsigned int>(start_orientation_bin);
  _a_star->setStart(mx_start, my_start, start_orientation_bin_int);

  unsigned int goal_orientation_bin_int = 0;
  if (_find_free_space_mode) {
    // Ignore the goal: fan out from the start and stop at the cheapest-to-reach
    // pose outside of the no-waiting zone
    _a_star->enableFreeSpaceSearch(
      _no_waiting_zone.createFreeSpaceStopChecker(
        costmap, _global_frame, _costmap_ros->getRobotFootprint()));
  } else {
    _a_star->disableFreeSpaceSearch();

    // Set goal point, in A* bin search coordinates
    if (!costmap->worldToMapContinuous(
        goal.pose.position.x,
        goal.pose.position.y,
        mx_goal,
        my_goal))
    {
      throw nav2_core::GoalOutsideMapBounds(
              "Goal Coordinates of(" + std::to_string(goal.pose.position.x) + ", " +
              std::to_string(goal.pose.position.y) + ") was outside bounds");
    }
    double goal_orientation_bin = std::round(tf2::getYaw(goal.pose.orientation) / _angle_bin_size);
    while (goal_orientation_bin < 0.0) {
      goal_orientation_bin += static_cast<float>(_angle_quantizations);
    }
    // This is needed to handle precision issues
    if (goal_orientation_bin >= static_cast<float>(_angle_quantizations)) {
      goal_orientation_bin -= static_cast<float>(_angle_quantizations);
    }
    goal_orientation_bin_int = static_cast<unsigned int>(goal_orientation_bin);
    _a_star->setGoal(
      mx_goal, my_goal, static_cast<unsigned int>(goal_orientation_bin_int),
      _goal_heading_mode, _coarse_search_resolution);
  }

  // Setup message
  nav_msgs::msg::Path plan;
  plan.header.stamp = _clock->now();
  plan.header.frame_id = _global_frame;
  geometry_msgs::msg::PoseStamped pose;
  pose.header = plan.header;
  pose.pose.position.z = 0.0;
  pose.pose.orientation.x = 0.0;
  pose.pose.orientation.y = 0.0;
  pose.pose.orientation.z = 0.0;
  pose.pose.orientation.w = 1.0;

  // Corner case of start and goal being on the same cell
  if (!_find_free_space_mode &&
    std::floor(mx_start) == std::floor(mx_goal) &&
    std::floor(my_start) == std::floor(my_goal) &&
    start_orientation_bin_int == goal_orientation_bin_int)
  {
    pose.pose = start.pose;
    pose.pose.orientation = goal.pose.orientation;
    plan.poses.push_back(pose);

    // Publish raw path for debug
    if (_raw_plan_publisher->get_subscription_count() > 0) {
      auto msg = std::make_unique<nav_msgs::msg::Path>(plan);
      _raw_plan_publisher->publish(std::move(msg));
    }

    return plan;
  }

  // Compute plan
  NodeHybrid::CoordinateVector path;
  int num_iterations = 0;
  std::string error;
  std::unique_ptr<std::vector<std::tuple<float, float, float>>> expansions = nullptr;
  if (_debug_visualizations) {
    expansions = std::make_unique<std::vector<std::tuple<float, float, float>>>();
  }
  // Note: All exceptions thrown are handled by the planner server and returned to the action
  if (!_a_star->createPath(
      path, num_iterations,
      _tolerance / static_cast<float>(costmap->getResolution()), cancel_checker, expansions.get()))
  {
    if (_debug_visualizations) {
      auto msg = std::make_unique<geometry_msgs::msg::PoseArray>();
      geometry_msgs::msg::Pose msg_pose;
      msg->header.stamp = _clock->now();
      msg->header.frame_id = _global_frame;
      for (auto & e : *expansions) {
        msg_pose.position.x = std::get<0>(e);
        msg_pose.position.y = std::get<1>(e);
        msg_pose.orientation = getWorldOrientation(std::get<2>(e));
        msg->poses.push_back(msg_pose);
      }
      _expansions_publisher->publish(std::move(msg));
    }

    // Note: If the start is blocked only one iteration will occur before failure
    if (num_iterations == 1) {
      throw nav2_core::StartOccupied("Start occupied");
    }

    // Only on request: callers that expect failures (e.g. trying many approach poses)
    // do not care why a search failed
    const bool search_exhausted = num_iterations < _a_star->getMaxIterations();
    const bool publish = _publish_failed_search && options.publish_failed_search;
    // Only when the search exhausted the reachable space: after max iterations the
    // explored pose closest to the goal is just where the search budget ran out
    const bool compute_partial_path =
      options.compute_partial_path && search_exhausted && !_find_free_space_mode;
    nav2_core::PartialPlan partial_plan;
    bool has_partial_plan = false;
    if (publish || compute_partial_path) {
      const FailedSearch search = analyzeFailedSearch(
        costmap, mx_start, my_start, mx_goal, my_goal);
      if (publish) {
        publishFailedSearch(
          costmap, search, duration_cast<duration<double>>(steady_clock::now() - a).count());
      }
      if (compute_partial_path) {
        has_partial_plan = computePartialPlan(costmap, search, partial_plan);
        partial_plan.path.header = plan.header;
        for (auto & partial_pose : partial_plan.path.poses) {
          partial_pose.header = plan.header;
        }
        if (_failed_partial_path_publisher &&
          _failed_partial_path_publisher->get_subscription_count() > 0)
        {
          // Also when there is none, so a stale one does not linger
          _failed_partial_path_publisher->publish(
            std::make_unique<nav_msgs::msg::Path>(partial_plan.path));
        }
      }
    }

    if (search_exhausted) {
      if (_find_free_space_mode) {
        throw nav2_core::NoValidPathCouldBeFound(
                "no reachable pose outside the no-waiting zone found");
      }
      if (has_partial_plan) {
        throw nav2_core::NoValidPathWithPartialPlan("no valid path found", partial_plan);
      }
      throw nav2_core::NoValidPathCouldBeFound("no valid path found");
    } else {
      throw nav2_core::PlannerTimedOut("exceeded maximum iterations");
    }
  }

  // Convert to world coordinates
  plan.poses.reserve(path.size());
  for (int i = path.size() - 1; i >= 0; --i) {
    pose.pose = getWorldCoords(path[i].x, path[i].y, costmap);
    pose.pose.orientation = getWorldOrientation(path[i].theta);
    plan.poses.push_back(pose);
  }

  // Publish raw path for debug
  if (_raw_plan_publisher->get_subscription_count() > 0) {
    auto msg = std::make_unique<nav_msgs::msg::Path>(plan);
    _raw_plan_publisher->publish(std::move(msg));
  }

  if (_debug_visualizations) {
    // Publish expansions for debug
    auto now = _clock->now();
    auto msg = std::make_unique<geometry_msgs::msg::PoseArray>();
    geometry_msgs::msg::Pose msg_pose;
    msg->header.stamp = now;
    msg->header.frame_id = _global_frame;
    for (auto & e : *expansions) {
      msg_pose.position.x = std::get<0>(e);
      msg_pose.position.y = std::get<1>(e);
      msg_pose.orientation = getWorldOrientation(std::get<2>(e));
      msg->poses.push_back(msg_pose);
    }
    _expansions_publisher->publish(std::move(msg));

    if (_planned_footprints_publisher->get_subscription_count() > 0) {
      // Clear all markers first
      auto marker_array = std::make_unique<visualization_msgs::msg::MarkerArray>();
      visualization_msgs::msg::Marker clear_all_marker;
      clear_all_marker.action = visualization_msgs::msg::Marker::DELETEALL;
      marker_array->markers.push_back(clear_all_marker);
      _planned_footprints_publisher->publish(std::move(marker_array));

      // Publish smoothed footprints for debug
      marker_array = std::make_unique<visualization_msgs::msg::MarkerArray>();
      for (size_t i = 0; i < plan.poses.size(); i++) {
        const std::vector<geometry_msgs::msg::Point> edge =
          transformFootprintToEdges(plan.poses[i].pose, _costmap_ros->getRobotFootprint());
        marker_array->markers.push_back(createMarker(edge, i, _global_frame, now));
      }
      _planned_footprints_publisher->publish(std::move(marker_array));
    }
  }

  // Find how much time we have left to do smoothing
  steady_clock::time_point b = steady_clock::now();
  duration<double> time_span = duration_cast<duration<double>>(b - a);
  double time_remaining = _max_planning_time - static_cast<double>(time_span.count());

#ifdef BENCHMARK_TESTING
  std::cout << "It took " << time_span.count() * 1000 <<
    " milliseconds with " << num_iterations << " iterations." << std::endl;
#endif

  // Smooth plan
  if (_smoother && num_iterations > 1) {
    _smoother->smooth(plan, costmap, time_remaining);
  }

#ifdef BENCHMARK_TESTING
  steady_clock::time_point c = steady_clock::now();
  duration<double> time_span2 = duration_cast<duration<double>>(c - b);
  std::cout << "It took " << time_span2.count() * 1000 <<
    " milliseconds to smooth path." << std::endl;
#endif

  if (_debug_visualizations) {
    if (_smoothed_footprints_publisher->get_subscription_count() > 0) {
      // Clear all markers first
      auto marker_array = std::make_unique<visualization_msgs::msg::MarkerArray>();
      visualization_msgs::msg::Marker clear_all_marker;
      clear_all_marker.action = visualization_msgs::msg::Marker::DELETEALL;
      marker_array->markers.push_back(clear_all_marker);
      _smoothed_footprints_publisher->publish(std::move(marker_array));

      // Publish smoothed footprints for debug
      marker_array = std::make_unique<visualization_msgs::msg::MarkerArray>();
      auto now = _clock->now();
      for (size_t i = 0; i < plan.poses.size(); i++) {
        const std::vector<geometry_msgs::msg::Point> edge =
          transformFootprintToEdges(plan.poses[i].pose, _costmap_ros->getRobotFootprint());
        marker_array->markers.push_back(createMarker(edge, i, _global_frame, now));
      }
      _smoothed_footprints_publisher->publish(std::move(marker_array));
    }
  }

  return plan;
}

unsigned int SmacPlannerHybrid::FailedSearch::headingsAt(
  const float & mx, const float & my) const
{
  if (mx < 0.0f || my < 0.0f) {
    return 0;
  }
  const unsigned int x = static_cast<unsigned int>(mx);
  const unsigned int y = static_cast<unsigned int>(my);
  if (x < min_x || y < min_y || x >= min_x + width || y >= min_y + height) {
    return 0;
  }
  return headings[static_cast<size_t>(y - min_y) * width + (x - min_x)];
}

float SmacPlannerHybrid::FailedSearch::costToGoAt(const float & mx, const float & my) const
{
  return cost_to_go[static_cast<size_t>(my / 2.0f) * cost_to_go_size_x +
           static_cast<size_t>(mx / 2.0f)];
}

SmacPlannerHybrid::FailedSearch SmacPlannerHybrid::analyzeFailedSearch(
  const nav2_costmap_2d::Costmap2D * costmap, const float & mx_start, const float & my_start,
  const float & mx_goal, const float & my_goal)
{
  FailedSearch search;

  // Pass 1: bounding box of the explored cells
  unsigned int min_x = std::numeric_limits<unsigned int>::max(), min_y = min_x;
  unsigned int max_x = 0, max_y = 0;
  _a_star->forEachVisitedNode(
    [&](NodeHybrid * node) {
      const unsigned int x = static_cast<unsigned int>(node->pose.x);
      const unsigned int y = static_cast<unsigned int>(node->pose.y);
      min_x = std::min(min_x, x);
      min_y = std::min(min_y, y);
      max_x = std::max(max_x, x);
      max_y = std::max(max_y, y);
      search.num_poses++;
    });
  if (search.num_poses == 0) {
    return search;
  }

  // Pass 2: number of headings the search reached in each explored cell
  search.min_x = min_x;
  search.min_y = min_y;
  search.width = max_x - min_x + 1;
  search.height = max_y - min_y + 1;
  search.headings.assign(static_cast<size_t>(search.width) * search.height, 0);
  _a_star->forEachVisitedNode(
    [&](NodeHybrid * node) {
      search.headings[(static_cast<size_t>(node->pose.y) - min_y) * search.width +
      (static_cast<size_t>(node->pose.x) - min_x)]++;
    });

  if (_find_free_space_mode) {
    return search;
  }

  // Pass 3: the explored pose closest to the goal, around blocked space rather than as the
  // crow flies (that can be a pocket behind a rack next to the goal)
  computeCostToGo(costmap, mx_start, my_start, mx_goal, my_goal, search);
  float closest_cost = std::numeric_limits<float>::max();
  _a_star->forEachVisitedNode(
    [&](NodeHybrid * node) {
      const float cost = search.costToGoAt(node->pose.x, node->pose.y);
      if (cost < closest_cost) {
        closest_cost = cost;
        search.closest = node;
      }
    });
  search.closest_goal_distance =
    std::hypot(search.closest->pose.x - mx_goal, search.closest->pose.y - my_goal);
  return search;
}

void SmacPlannerHybrid::computeCostToGo(
  const nav2_costmap_2d::Costmap2D * costmap, const float & mx_start, const float & my_start,
  const float & mx_goal, const float & my_goal, FailedSearch & search)
{
  const unsigned int costmap_size_x = costmap->getSizeInCellsX();
  const unsigned int costmap_size_y = costmap->getSizeInCellsY();
  const unsigned int size_x = (costmap_size_x + 1) / 2;
  const unsigned int size_y = (costmap_size_y + 1) / 2;
  const size_t size = static_cast<size_t>(size_x) * size_y;
  search.cost_to_go_size_x = size_x;

  // Travel cost per meter of each cell: 1, or the blocked factor if even the least costly of
  // its 2x2 costmap cells puts the robot center into collision
  const float blocked_factor = static_cast<float>(_partial_path_blocked_cost_factor);
  std::vector<float> factor(size, 1.0f);
  for (unsigned int y = 0; y < size_y; ++y) {
    for (unsigned int x = 0; x < size_x; ++x) {
      unsigned char min_cost = nav2_costmap_2d::NO_INFORMATION;
      for (unsigned int j = 2 * y; j < std::min(2 * y + 2, costmap_size_y); ++j) {
        for (unsigned int i = 2 * x; i < std::min(2 * x + 2, costmap_size_x); ++i) {
          min_cost = std::min(min_cost, costmap->getCost(i, j));
        }
      }
      if (min_cost >= nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE) {
        factor[static_cast<size_t>(y) * size_x + x] = blocked_factor;
      }
    }
  }

  const auto index = [size_x](const float & mx, const float & my) {
      return static_cast<size_t>(my / 2.0f) * size_x + static_cast<size_t>(mx / 2.0f);
    };

  // Cells whose cost is needed: the start and every explored cell
  std::vector<uint8_t> needed(size, 0);
  size_t num_needed = 0;
  const auto mark = [&](const float & mx, const float & my) {
      const size_t i = index(mx, my);
      if (!needed[i]) {
        needed[i] = 1;
        num_needed++;
      }
    };
  mark(mx_start, my_start);
  _a_star->forEachVisitedNode([&](NodeHybrid * node) {mark(node->pose.x, node->pose.y);});

  // Dijkstra from the goal until all of them are settled
  const float step = 2.0f * static_cast<float>(costmap->getResolution());
  const float diagonal_step = step * std::sqrt(2.0f);
  std::vector<float> & cost = search.cost_to_go;
  cost.assign(size, std::numeric_limits<float>::max());
  std::vector<uint8_t> closed(size, 0);
  using QueueEntry = std::pair<float, size_t>;
  std::priority_queue<QueueEntry, std::vector<QueueEntry>, std::greater<QueueEntry>> queue;
  const size_t goal_index = index(mx_goal, my_goal);
  cost[goal_index] = 0.0f;
  queue.emplace(0.0f, goal_index);
  while (!queue.empty() && num_needed > 0) {
    const auto [current_cost, current] = queue.top();
    queue.pop();
    if (closed[current]) {
      continue;
    }
    closed[current] = 1;
    if (needed[current]) {
      num_needed--;
    }
    const int cx = static_cast<int>(current % size_x);
    const int cy = static_cast<int>(current / size_x);
    for (int ny = cy - 1; ny <= cy + 1; ++ny) {
      for (int nx = cx - 1; nx <= cx + 1; ++nx) {
        if ((nx == cx && ny == cy) || nx < 0 || ny < 0 ||
          nx >= static_cast<int>(size_x) || ny >= static_cast<int>(size_y))
        {
          continue;
        }
        const size_t neighbor = static_cast<size_t>(ny) * size_x + static_cast<size_t>(nx);
        if (closed[neighbor]) {
          continue;
        }
        const float length = (nx != cx && ny != cy) ? diagonal_step : step;
        const float new_cost =
          current_cost + length * 0.5f * (factor[current] + factor[neighbor]);
        if (new_cost < cost[neighbor]) {
          cost[neighbor] = new_cost;
          queue.emplace(new_cost, neighbor);
        }
      }
    }
  }

  search.start_cost_to_go = search.costToGoAt(mx_start, my_start);
}

void SmacPlannerHybrid::publishFailedSearch(
  const nav2_costmap_2d::Costmap2D * costmap, const FailedSearch & search,
  const double & search_duration)
{
  if (search.num_poses == 0) {
    return;
  }
  const size_t num_cells = static_cast<size_t>(
    std::count_if(
      search.headings.begin(), search.headings.end(), [](unsigned int n) {return n > 0;}));

  const double resolution = costmap->getResolution();
  if (_find_free_space_mode) {
    RCLCPP_WARN(
      _logger, "%s: search explored %zu poses in %zu cells (%.1f m^2) within %.3f s",
      _name.c_str(), search.num_poses, num_cells, num_cells * resolution * resolution,
      search_duration);
  } else {
    const geometry_msgs::msg::Pose closest_pose =
      getWorldCoords(search.closest->pose.x, search.closest->pose.y, costmap);
    RCLCPP_WARN(
      _logger, "%s: search explored %zu poses in %zu cells (%.1f m^2) within %.3f s, "
      "closest pose (%.2f, %.2f) is %.2f m from the goal",
      _name.c_str(), search.num_poses, num_cells, num_cells * resolution * resolution,
      search_duration, closest_pose.position.x, closest_pose.position.y,
      search.closest_goal_distance * resolution);
  }

  const auto now = _clock->now();
  if (_failed_explored_area_publisher->get_subscription_count() > 0) {
    // 0 = not reached (transparent in the costmap color scheme), 1 = reached in every
    // heading up to 98 = reached in a single heading only, so cells the robot passes but
    // cannot turn in stand out
    auto grid = std::make_unique<nav_msgs::msg::OccupancyGrid>();
    grid->header.stamp = now;
    grid->header.frame_id = _global_frame;
    grid->info.map_load_time = now;
    grid->info.resolution = static_cast<float>(resolution);
    grid->info.width = search.width;
    grid->info.height = search.height;
    grid->info.origin.position.x = costmap->getOriginX() + search.min_x * resolution;
    grid->info.origin.position.y = costmap->getOriginY() + search.min_y * resolution;
    grid->info.origin.orientation.w = 1.0;
    grid->data.resize(search.headings.size());
    for (size_t i = 0; i < search.headings.size(); ++i) {
      if (search.headings[i] == 0) {
        grid->data[i] = 0;
        continue;
      }
      const double share =
        std::min(1.0, static_cast<double>(search.headings[i]) / _angle_quantizations);
      grid->data[i] = static_cast<int8_t>(1 + std::lround(97.0 * (1.0 - share)));
    }
    _failed_explored_area_publisher->publish(std::move(grid));
  }

  if (_failed_closest_path_publisher->get_subscription_count() > 0) {
    // Start -> explored pose closest to the goal; empty in find free space mode, which
    // ignores the goal, so a stale path of an earlier failure does not linger
    auto path_msg = std::make_unique<nav_msgs::msg::Path>();
    path_msg->header.stamp = now;
    path_msg->header.frame_id = _global_frame;
    NodeHybrid::CoordinateVector path;
    if (!_find_free_space_mode && search.closest->backtracePath(path)) {
      geometry_msgs::msg::PoseStamped pose;
      pose.header = path_msg->header;
      path_msg->poses.reserve(path.size());
      for (int i = path.size() - 1; i >= 0; --i) {
        pose.pose = getWorldCoords(path[i].x, path[i].y, costmap);
        pose.pose.orientation = getWorldOrientation(path[i].theta);
        path_msg->poses.push_back(pose);
      }
    }
    _failed_closest_path_publisher->publish(std::move(path_msg));
  }
}

bool SmacPlannerHybrid::computePartialPlan(
  const nav2_costmap_2d::Costmap2D * costmap, const FailedSearch & search,
  nav2_core::PartialPlan & partial_plan)
{
  NodeHybrid::CoordinateVector path;
  if (!search.closest || search.cost_to_go.empty() || !search.closest->backtracePath(path)) {
    // Nothing explored beyond the start
    return false;
  }
  std::reverse(path.begin(), path.end());  // start -> closest pose

  // Only the first driving direction: waiting at the end of a K-turn or of a reverse move
  // into a pocket leaves the robot badly placed for whatever comes next
  size_t end = path.size() - 1;
  if (!_partial_path_allow_reversing) {
    for (size_t i = 1; i < path.size(); ++i) {
      const float dx = path[i].x - path[i - 1].x;
      const float dy = path[i].y - path[i - 1].y;
      if (dx * std::cos(path[i].theta) + dy * std::sin(path[i].theta) < 0.0f) {
        end = i - 1;
        break;
      }
    }
  }

  // Stop short of the blockage: at the end of the search the footprint only just fits next
  // to it
  const double resolution = costmap->getResolution();
  std::vector<double> length(end + 1, 0.0);
  for (size_t i = 1; i <= end; ++i) {
    length[i] = length[i - 1] +
      std::hypot(path[i].x - path[i - 1].x, path[i].y - path[i - 1].y) * resolution;
  }
  size_t last = end;
  while (last > 0 && length[end] - length[last] < _partial_path_backoff) {
    last--;
  }
  // ... in a cell the robot can still turn in a bit
  while (last > 0 &&
    search.headingsAt(path[last].x, path[last].y) <
    static_cast<unsigned int>(_partial_path_min_end_headings))
  {
    last--;
  }
  if (last == 0) {
    return false;
  }

  const float end_cost_to_go = search.costToGoAt(path[last].x, path[last].y);
  if (end_cost_to_go >= search.start_cost_to_go) {
    return false;
  }

  partial_plan.path.poses.clear();
  partial_plan.path.poses.reserve(last + 1);
  geometry_msgs::msg::PoseStamped pose;
  for (size_t i = 0; i <= last; ++i) {
    pose.pose = getWorldCoords(path[i].x, path[i].y, costmap);
    pose.pose.orientation = getWorldOrientation(path[i].theta);
    partial_plan.path.poses.push_back(pose);
  }
  partial_plan.start_cost_to_go = search.start_cost_to_go;
  partial_plan.end_cost_to_go = end_cost_to_go;

  RCLCPP_WARN(
    _logger, "%s: partial path of %.2f m to (%.2f, %.2f), %.2f m short of the explored "
    "pose closest to the goal; distance to the goal %.2f m -> %.2f m",
    _name.c_str(), length[last], partial_plan.path.poses.back().pose.position.x,
    partial_plan.path.poses.back().pose.position.y, length[end] - length[last],
    partial_plan.start_cost_to_go, partial_plan.end_cost_to_go);
  return true;
}

rcl_interfaces::msg::SetParametersResult SmacPlannerHybrid::validateParameterUpdatesCallback(
  const std::vector<rclcpp::Parameter> & parameters)
{
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;
  for (const auto & parameter : parameters) {
    const auto & param_type = parameter.get_type();
    const auto & param_name = parameter.get_name();
    if (param_name.find(_name + ".") != 0) {
      continue;
    }
    if (param_type == ParameterType::PARAMETER_DOUBLE) {
      if (parameter.as_double() < 0.0) {
        RCLCPP_WARN(
        _logger, "The value of parameter '%s' is incorrectly set to %f, "
        "it should be >=0. Ignoring parameter update.",
        param_name.c_str(), parameter.as_double());
        result.successful = false;
      } else if (param_name == _name + ".minimum_turning_radius" && // NOLINT
        parameter.as_double() < _costmap->getResolution() * _downsampling_factor)
      {
        RCLCPP_WARN(
          _logger, "The value of parameter minimum_turning_radius is incorrectly set to %f, "
          "it should be >= costmap resolution * downsampling factor (%f). "
          "Ignoring parameter update.",
          parameter.as_double(),
          _costmap->getResolution() * _downsampling_factor);
        result.successful = false;
      } else if (param_name == _name + ".minimum_turning_radius_right" && // NOLINT
        parameter.as_double() > 0.0 &&
        parameter.as_double() < _costmap->getResolution() * _downsampling_factor)
      {
        RCLCPP_WARN(
          _logger, "The value of parameter minimum_turning_radius_right is incorrectly set to %f, "
          "it should be >= costmap resolution * downsampling factor (%f) or 0.0 to disable. "
          "Ignoring parameter update.",
          parameter.as_double(),
          _costmap->getResolution() * _downsampling_factor);
        result.successful = false;
      }
    } else if (param_type == ParameterType::PARAMETER_INTEGER) {
      if (parameter.as_int() <= 0 && (param_name != _name + ".max_on_approach_iterations" && // NOLINT
        param_name != _name + ".max_iterations"))
      {
        RCLCPP_WARN(
        _logger, "The value of parameter '%s' is incorrectly set to %ld, "
        "it should be >0. Ignoring parameter update.",
        param_name.c_str(), parameter.as_int());
        result.successful = false;
      } else if (param_name == _name + ".angle_quantization_bins") {
        unsigned int angle_quantizations = static_cast<unsigned int>(parameter.as_int());
        if (angle_quantizations % _coarse_search_resolution != 0) {
          RCLCPP_WARN(
            _logger,
            "The value of parameter angle_quantization_bins is incorrectly set to %u, "
            "it should be an increment of the coarse_search_resolution (%u). "
            "Ignoring parameter update.",
            angle_quantizations,
            _coarse_search_resolution);
          result.successful = false;
        }
      } else if (param_name == _name + ".coarse_search_resolution") {
        if (_angle_quantizations % static_cast<unsigned int>(parameter.as_int()) != 0) {
          RCLCPP_WARN(
            _logger,
            "The value of parameter coarse_search_resolution is incorrectly set to %ld, "
            "it should be an increment of the angle_quantization_bins (%u). "
            "Ignoring parameter update.",
            parameter.as_int(),
            _angle_quantizations);
          result.successful = false;
        }
      }
    } else if (param_type == ParameterType::PARAMETER_STRING) {
      if (param_name == _name + ".motion_model_for_search") {
        MotionModel motion_model = fromString(parameter.as_string());
        if (motion_model == MotionModel::UNKNOWN) {
          RCLCPP_WARN(
            _logger,
            "Unable to get MotionModel search type. Given '%s', "
            "valid options are MOORE, VON_NEUMANN, DUBIN, REEDS_SHEPP, STATE_LATTICE. "
            "Ignoring parameter update.",
            parameter.as_string().c_str());
          result.successful = false;
        }
      } else if (param_name == _name + ".goal_heading_mode") {
        GoalHeadingMode goal_heading_mode = fromStringToGH(parameter.as_string());
        if (goal_heading_mode == GoalHeadingMode::UNKNOWN) {
          RCLCPP_WARN(
            _logger,
            "Unable to get GoalHeader type. Given '%s' Valid options are DEFAULT, "
            "BIDIRECTIONAL, ALL_DIRECTION. Ignoring parameter update.",
            parameter.as_string().c_str());
          result.successful = false;
        }
      }
    }
  }
  return result;
}

void
SmacPlannerHybrid::updateParametersCallback(const std::vector<rclcpp::Parameter> & parameters)
{
  std::lock_guard<std::mutex> lock_reinit(_mutex);

  bool reinit_collision_checker = false;
  bool reinit_a_star = false;
  bool reinit_lookup_table = false;
  bool reinit_downsampler = false;
  bool reinit_smoother = false;

  for (auto parameter : parameters) {
    const auto & param_type = parameter.get_type();
    const auto & param_name = parameter.get_name();
    if (param_name.find(_name + ".") != 0 && param_name != "resolution") {
      continue;
    }
    if (param_type == ParameterType::PARAMETER_DOUBLE) {
      if (param_name == _name + ".max_planning_time") {
        reinit_a_star = true;
        _max_planning_time = parameter.as_double();
      } else if (param_name == _name + ".tolerance") {
        _tolerance = static_cast<float>(parameter.as_double());
      } else if (param_name == _name + ".lookup_table_size") {
        reinit_a_star = true;
        reinit_lookup_table = true;
        _lookup_table_size = parameter.as_double();
      } else if (param_name == _name + ".minimum_turning_radius") {
        reinit_a_star = true;
        reinit_lookup_table = true;
        if (_smoother) {
          reinit_smoother = true;
        }
        _minimum_turning_radius_global_coords = static_cast<float>(parameter.as_double());
      } else if (param_name == _name + ".minimum_turning_radius_right") {
        reinit_a_star = true;
        reinit_lookup_table = true;
        if (_smoother) {
          reinit_smoother = true;
        }
        _minimum_turning_radius_right_global_coords =
          static_cast<float>(parameter.as_double());
      } else if (param_name == _name + ".reverse_penalty") {
        reinit_a_star = true;
        _search_info.reverse_penalty = static_cast<float>(parameter.as_double());
      } else if (param_name == _name + ".change_penalty") {
        reinit_a_star = true;
        _search_info.change_penalty = static_cast<float>(parameter.as_double());
      } else if (param_name == _name + ".non_straight_penalty") {
        reinit_a_star = true;
        _search_info.non_straight_penalty = static_cast<float>(parameter.as_double());
      } else if (param_name == _name + ".cost_penalty") {
        reinit_a_star = true;
        _search_info.cost_penalty = static_cast<float>(parameter.as_double());
      } else if (param_name == _name + ".analytic_expansion_ratio") {
        reinit_a_star = true;
        _search_info.analytic_expansion_ratio = static_cast<float>(parameter.as_double());
      } else if (param_name == _name + ".analytic_expansion_max_length") {
        reinit_a_star = true;
        _search_info.analytic_expansion_max_length =
          static_cast<float>(parameter.as_double()) / _costmap->getResolution();
      } else if (param_name == _name + ".analytic_expansion_max_cost") {
        reinit_a_star = true;
        _search_info.analytic_expansion_max_cost = static_cast<float>(parameter.as_double());
      } else if (param_name == "resolution") {
        // Special case: When the costmap's resolution changes, need to reinitialize
        // the controller to have new resolution information
        RCLCPP_INFO(_logger, "Costmap resolution changed. Reinitializing SmacPlannerHybrid.");
        reinit_collision_checker = true;
        reinit_a_star = true;
        reinit_lookup_table = true;
        reinit_downsampler = true;
        reinit_smoother = true;
      }
    } else if (param_type == ParameterType::PARAMETER_BOOL) {
      if (param_name == _name + ".downsample_costmap") {
        reinit_downsampler = true;
        _downsample_costmap = parameter.as_bool();
      } else if (param_name == _name + ".allow_unknown") {
        reinit_a_star = true;
        _allow_unknown = parameter.as_bool();
      } else if (param_name == _name + ".cache_obstacle_heuristic") {
        reinit_a_star = true;
        _search_info.cache_obstacle_heuristic = parameter.as_bool();
      } else if (param_name == _name + ".allow_primitive_interpolation") {
        _search_info.allow_primitive_interpolation = parameter.as_bool();
        reinit_a_star = true;
      } else if (param_name == _name + ".smooth_path") {
        if (parameter.as_bool()) {
          reinit_smoother = true;
        } else {
          _smoother.reset();
        }
      } else if (param_name == _name + ".analytic_expansion_max_cost_override") {
        _search_info.analytic_expansion_max_cost_override = parameter.as_bool();
        reinit_a_star = true;
      }
    } else if (param_type == ParameterType::PARAMETER_INTEGER) {
      if (param_name == _name + ".downsampling_factor") {
        reinit_a_star = true;
        reinit_lookup_table = true;
        reinit_downsampler = true;
        _downsampling_factor = parameter.as_int();
      } else if (param_name == _name + ".max_iterations") {
        reinit_a_star = true;
        _max_iterations = parameter.as_int();
        if (_max_iterations <= 0) {
          RCLCPP_INFO(
            _logger, "maximum iteration selected as <= 0, "
            "disabling maximum iterations.");
          _max_iterations = std::numeric_limits<int>::max();
        }
      } else if (param_name == _name + ".max_on_approach_iterations") {
        reinit_a_star = true;
        _max_on_approach_iterations = parameter.as_int();
        if (_max_on_approach_iterations <= 0) {
          RCLCPP_INFO(
            _logger, "On approach iteration selected as <= 0, "
            "disabling tolerance and on approach iterations.");
          _max_on_approach_iterations = std::numeric_limits<int>::max();
        }
      } else if (param_name == _name + ".terminal_checking_interval") {
        reinit_a_star = true;
        _terminal_checking_interval = parameter.as_int();
      } else if (param_name == _name + ".angle_quantization_bins") {
        reinit_collision_checker = true;
        reinit_a_star = true;
        reinit_lookup_table = true;
        int angle_quantizations = parameter.as_int();
        _angle_bin_size = 2.0 * M_PI / angle_quantizations;
        _angle_quantizations = static_cast<unsigned int>(angle_quantizations);
      } else if (param_name == _name + ".coarse_search_resolution") {
        _coarse_search_resolution = parameter.as_int();
      }
    } else if (param_type == ParameterType::PARAMETER_STRING) {
      if (param_name == _name + ".motion_model_for_search") {
        reinit_a_star = true;
        reinit_lookup_table = true;
        _motion_model = fromString(parameter.as_string());
      } else if (param_name == _name + ".goal_heading_mode") {
        std::string goal_heading_type = parameter.as_string();
        GoalHeadingMode goal_heading_mode = fromStringToGH(goal_heading_type);
        RCLCPP_INFO(
          _logger,
          "GoalHeadingMode type set to '%s'.",
          goal_heading_type.c_str());
        _goal_heading_mode = goal_heading_mode;
      }
    }
  }

  // Re-init if needed with mutex lock (to avoid re-init while creating a plan)
  if (reinit_a_star || reinit_downsampler || reinit_collision_checker || reinit_smoother) {
    // convert to grid coordinates
    if (!_downsample_costmap) {
      _downsampling_factor = 1;
    }
    _search_info.minimum_turning_radius =
      _minimum_turning_radius_global_coords / (_costmap->getResolution() * _downsampling_factor);
    _search_info.minimum_turning_radius_right =
      _minimum_turning_radius_right_global_coords > 0.0 ?
      _minimum_turning_radius_right_global_coords /
      (_costmap->getResolution() * _downsampling_factor) :
      0.0f;
    _lookup_table_dim =
      static_cast<float>(_lookup_table_size) /
      static_cast<float>(_costmap->getResolution() * _downsampling_factor);

    // Make sure its a whole number
    _lookup_table_dim = static_cast<float>(static_cast<int>(_lookup_table_dim));

    // Make sure its an odd number
    if (static_cast<int>(_lookup_table_dim) % 2 == 0) {
      RCLCPP_INFO(
        _logger,
        "Even sized heuristic lookup table size set %f, increasing size by 1 to make odd",
        _lookup_table_dim);
      _lookup_table_dim += 1.0;
    }

    auto node = _node.lock();

    // Re-Initialize A* template
    if (reinit_a_star) {
      if (reinit_lookup_table) {
        _a_star = std::make_unique<AStarAlgorithm<NodeHybrid>>(_motion_model, _search_info);
      } else {
        _a_star->setSearchInfo(_search_info);
      }
      _a_star->initialize(
        _allow_unknown,
        _max_iterations,
        _max_on_approach_iterations,
        _terminal_checking_interval,
        _max_planning_time,
        _lookup_table_dim,
        _angle_quantizations);
    }

    // Re-Initialize costmap downsampler
    if (reinit_downsampler) {
      if (_downsample_costmap && _downsampling_factor > 1) {
        std::string topic_name = "downsampled_costmap";
        _costmap_downsampler = std::make_unique<CostmapDownsampler>();
        _costmap_downsampler->on_configure(
          node, _global_frame, topic_name, _costmap, _downsampling_factor);
        _costmap_downsampler->on_activate();
      }
    }

    // Re-Initialize collision checker
    if (reinit_collision_checker) {
      _collision_checker = GridCollisionChecker(_costmap_ros, _angle_quantizations, node);
      _collision_checker.setFootprint(
        _costmap_ros->getRobotFootprint(),
        _costmap_ros->getUseRadius(),
        findCircumscribedCost(_costmap_ros));
    }

    // Re-Initialize smoother
    if (reinit_smoother) {
      const double smoother_radius = std::max(
        _minimum_turning_radius_global_coords, _minimum_turning_radius_right_global_coords);
      SmootherParams params;
      params.get(node, _name);
      _smoother = std::make_unique<Smoother>(params);
      _smoother->initialize(smoother_radius);
    }
  }
}

}  // namespace nav2_smac_planner

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(nav2_smac_planner::SmacPlannerHybrid, nav2_core::GlobalPlanner)
