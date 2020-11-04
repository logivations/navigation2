# Copyright (c) 2018 Intel Corporation
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node
from nav2_common.launch import RewrittenYaml

from launch_ros.actions import PushRosNamespace

def generate_launch_description():
    # Get the launch directory
    bringup_dir = get_package_share_directory('nav2_bringup')

    namespace = LaunchConfiguration('namespace')
    use_namespace = LaunchConfiguration('use_namespace')
    use_valgrind = LaunchConfiguration('use_valgrind')
    use_sim_time = LaunchConfiguration('use_sim_time')
    autostart = LaunchConfiguration('autostart')
    params_file = LaunchConfiguration('params_file')
    use_lifecycle_mgr = LaunchConfiguration('use_lifecycle_mgr')
    use_remappings = LaunchConfiguration('use_remappings')

    # TODO(orduno) Remove once `PushNodeRemapping` is resolved
    #              https://github.com/ros2/launch_ros/issues/56
    remappings = [('/tf', 'tf'),
                  ('/tf_static', 'tf_static')]

    # Create our own temporary YAML files that include substitutions
    param_substitutions = {
        'use_sim_time': use_sim_time,
        'autostart': autostart,}
    prefix = ''
    if use_valgrind == True:
        prefix = 'valgrind --track-origins=yes'

    configured_params = RewrittenYaml(
        source_file=params_file,
        root_key=namespace,
        param_rewrites=param_substitutions,
        convert_types=True)

    return LaunchDescription([
        # Set env var to print messages to stdout immediately
        SetEnvironmentVariable('RCUTILS_CONSOLE_STDOUT_LINE_BUFFERED', '1'),
        DeclareLaunchArgument(
            'use_namespace',
            default_value='false',
            description='Whether to apply a namespace to the navigation stack'),
        DeclareLaunchArgument(
            'namespace', default_value='',
            description='Top-level namespace'),

        DeclareLaunchArgument(
            'use_sim_time', default_value='false',
            description='Use simulation (Gazebo) clock if true'),

        DeclareLaunchArgument(
            'autostart', default_value='true',
            description='Automatically startup the nav2 stack'),

        DeclareLaunchArgument(
            'params_file',
            default_value=os.path.join(bringup_dir, 'params', 'teb_params.yaml'),
            description='Full path to the ROS2 parameters file to use'),

        DeclareLaunchArgument(
            'use_lifecycle_mgr', default_value='true',
            description='Whether to launch the lifecycle manager'),

        DeclareLaunchArgument(
            'use_remappings', default_value='false',
            description='Arguments to pass to all nodes launched by the file'),
            
        DeclareLaunchArgument(
            'default_bt_xml_filename',
            default_value=os.path.join(
                get_package_share_directory('nav2_bt_navigator'),
                'behavior_trees', 'navigate_w_replanning_and_recovery.xml'),
            description='Full path to the behavior tree xml file to use'),

        PushRosNamespace(
             condition=IfCondition(use_namespace),
             namespace=namespace),
        Node(
            package='nav2_controller',
            executable='controller_server',
            output='screen',
            parameters=[configured_params],
            prefix=[prefix],
            #remappings=remappings
        ),
        
        Node(
            package='nav2_bt_navigator',
            executable='bt_navigator',
            name='bt_navigator',
            output='screen',
            parameters=[configured_params],
            arguments=['--ros-args', '--log-level', 'DEBUG'],
            remappings=remappings),

        Node(
            condition=IfCondition(use_lifecycle_mgr),
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_navigation',
            arguments=['--ros-args', '--log-level', 'DEBUG'],
            output='screen',
            parameters=[{'use_sim_time': use_sim_time},
                        {'autostart': autostart},
                        {'node_names': ['controller_server', 'bt_navigator']}]),

    ])