# Copyright (c) 2021 Juan Miguel Jimeno
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http:#www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.substitutions import PythonExpression
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node

package_name = 'wms_navigation'
pkg_share = FindPackageShare(package=package_name).find(package_name)

MAP_NAME='wms2' #change to the name of your own map here

def generate_launch_description():
    depth_sensor = os.getenv('LINOROBOT2_DEPTH_SENSOR', '')

    nav2_launch_path = PathJoinSubstitution(
        [FindPackageShare('nav2_bringup'), 'launch', 'bringup_launch.py']
    )

    rviz_config_path = PathJoinSubstitution(
        [pkg_share, 'rviz', 'wms_navigation.rviz']
    )

    default_map_path = PathJoinSubstitution(
        [pkg_share, 'maps', f'{MAP_NAME}.yaml']
    )

    nav2_config_path = PathJoinSubstitution(
        [pkg_share, 'config', 'navigation.yaml']
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            name='sim', 
            default_value='false',
            description='Enable use_sime_time to true'
        ),

        DeclareLaunchArgument(
            name='rviz', 
            default_value='false',
            description='Run rviz'
        ),
        
        DeclareLaunchArgument(
            name='slam',
            default_value='False',
            description='Whether to run SLAM'
        ),

        DeclareLaunchArgument(
            name='map', 
            default_value=default_map_path,
            description='Navigation map path'
        ),
        DeclareLaunchArgument(
            name='auto_slam', 
            default_value='False',
            description='Autonomous Slam according to SlamToolbox and Nav2'
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(nav2_launch_path),
            launch_arguments={
                'slam': LaunchConfiguration("slam"), # slam will eliminate map server (default_map_path)
                'map': LaunchConfiguration("map"),
                'use_sim_time': LaunchConfiguration("sim"),
                'params_file': nav2_config_path
            }.items()
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_config_path],
            condition=IfCondition(LaunchConfiguration("rviz")),
            parameters=[{'use_sim_time': LaunchConfiguration("sim")}]
        ),
        Node(
            # condition=IfCondition(LaunchConfiguration("auto_slam")),
            condition=IfCondition(PythonExpression([LaunchConfiguration("slam"), ' and ', LaunchConfiguration("auto_slam")])),
            package='wms_navigation',
            executable='map_analyzer.py',
            name='map_analyzer',
            # equals to: launch_arguments = {'params_file': params_file}.items(),
            parameters = [
                {'use_sim_time': LaunchConfiguration("sim")},
                PathJoinSubstitution([FindPackageShare('wms_navigation'), 'config', 'discovery_setting.yaml'])
                
            ],
            
            output='screen'
        ),
        Node(
            # condition=IfCondition(LaunchConfiguration("auto_slam")),
            condition=IfCondition(PythonExpression([LaunchConfiguration("slam"), ' and ', LaunchConfiguration("auto_slam")])),
            package='wms_navigation',
            executable='discovery_server.py',
            name='discovery_server',
            # equals to: launch_arguments = {'params_file': params_file}.items(),
            parameters = [
                {'use_sim_time': LaunchConfiguration("sim")},
                PathJoinSubstitution([FindPackageShare('wms_navigation'), 'config', 'discovery_setting.yaml']),
            ],
            output='screen'
        )
    ])
