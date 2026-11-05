nav2_add_gtest(test_controller_server
  test_controller_server.cpp
)
target_link_libraries(test_controller_server
  ${library_name}
  nav2_util::nav2_util_core
  rclcpp::rclcpp
)

find_package(rclcpp_action REQUIRED)
find_package(ament_index_cpp REQUIRED)

nav2_add_gtest(test_goal_reached_before_control
  test_goal_reached_before_control.cpp
)
target_link_libraries(test_goal_reached_before_control
  ${library_name}
  nav2_util::nav2_util_core
  rclcpp::rclcpp
  rclcpp_action::rclcpp_action
  tf2_ros::tf2_ros
  ament_index_cpp::ament_index_cpp
  ${nav2_msgs_TARGETS}
  ${geometry_msgs_TARGETS}
)
