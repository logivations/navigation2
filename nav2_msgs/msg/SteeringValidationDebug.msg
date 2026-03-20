# Debug output from VelocityPolygon::validateSteering
std_msgs/Header header

# Input velocities
float64 target_speed            # cmd_vel_in.x (baselink)
float64 current_speed           # odom_vel.x (baselink)
float64 target_steering_angle   # computed from cmd_vel_in
float64 current_steering_angle  # computed from odom_vel
float64 target_sw_speed         # target steering wheel speed
float64 current_sw_speed        # current steering wheel speed

# cmd_vel_in components
float64 cmd_vel_x
float64 cmd_vel_y
float64 cmd_vel_tw

# odom_vel components
float64 odom_vel_x
float64 odom_vel_y
float64 odom_vel_tw

# Step 1 info (LIMIT polygon collision check before steering validation)
string step1_active_sub_polygon # sub-polygon name active during step 1
int32 step1_points_inside       # collision points found inside the active polygon
int32 step1_min_points          # threshold for triggering action
bool step1_shape_set            # whether polygon shape was set
float64 step1_linear_limit      # linear limit of the active sub-polygon (baselink)
float64 step1_angular_limit     # angular limit of the active sub-polygon
uint8 step1_action_type         # action type after step 1 (0=DO_NOTHING,1=STOP,2=SLOWDOWN,3=APPROACH,4=LIMIT)
float64 step1_req_vel_x         # velocity x after step 1
float64 step1_req_vel_y         # velocity y after step 1
float64 step1_req_vel_tw        # velocity tw after step 1

# Source filtering diagnostics
string[] step1_configured_sources    # source names this polygon is configured to check
int32 step1_configured_source_pts    # total points from configured sources (before polygon test)
int32 step1_all_source_pts           # total points across ALL sources in the map
int32 step1_polygon_vertex_count     # number of vertices in the active polygon shape

# Bounding box diagnostics — compare polygon bounds vs collision point bounds
float64 step1_poly_min_x
float64 step1_poly_max_x
float64 step1_poly_min_y
float64 step1_poly_max_y
float64 step1_pts_min_x              # min x of all collision points from configured sources
float64 step1_pts_max_x
float64 step1_pts_min_y
float64 step1_pts_max_y
int32 step1_pts_in_bbox              # points within the polygon's bounding box (but not necessarily inside polygon)

# Step 2 decision path
bool crosses_zero               # direction reversal detected
bool both_below_threshold       # both speeds below low_speed_threshold
bool same_bucket                # target angle in same bucket as current
string current_field_name       # field matching current velocity ("" if none)
string valid_field_name         # collision-free neighbour field ("" if N/A)
string next_field_name          # one-step-faster field checked in same-bucket case

# Collision info
int32 next_field_collision_pts  # points inside next field (same-bucket case)
int32 neighbour_collision_pts   # points inside neighbour start field (diff-bucket case)

# Modification result
bool modified                   # whether velocity was modified by step 2
float64 result_vel_x            # final output velocity x
float64 result_vel_y            # final output velocity y
float64 result_vel_tw           # final output velocity tw

# Limits applied
float64 speed_limit_applied     # baselink speed limit applied (0 if none)
float64 steering_angle_limit    # steering angle limit applied (NaN if none)

# Polygon configuration
string polygon_name
float64 wheelbase               # wheelbase used for steering calculations
