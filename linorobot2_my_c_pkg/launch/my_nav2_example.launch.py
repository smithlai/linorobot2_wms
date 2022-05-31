# Author: Smith
# Date: May 31, 2022
# Description: This is a template example code to launch linobot nav2
# `ros2 launch linorobot2_gazebo gazebo.launch.py`
# `

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PythonExpression, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
  package_name = 'linorobot2_my_c_pkg'
     
  pkg_share = FindPackageShare(package=package_name).find(package_name)
  pkg_share2 = FindPackageShare('linorobot2_navigation') 
 
  # Declare the launch arguments  

  # Namespace
  declare_namespace_cmd = DeclareLaunchArgument(
    name='namespace',
    default_value='',
    description='Top-level namespace')

  declare_use_namespace_cmd = DeclareLaunchArgument(
    name='use_namespace',
    default_value='false',
    description='Whether to apply a namespace to the navigation stack')
  
  # Map
  default_map_path = PathJoinSubstitution(
        [pkg_share2, 'maps', 'playground.yaml']
    )
  declare_map_yaml_cmd = DeclareLaunchArgument(
    name='map',
    default_value=default_map_path,
    description='Full path to map file to load')

  # Sim
  declare_use_sim_time_cmd = DeclareLaunchArgument(
    name='use_sim_time',
    default_value='true',
    description='Use simulation (Gazebo) clock if true')

  # Rviz
  default_rviz_config_path = PathJoinSubstitution(
        [pkg_share2, 'rviz', 'linorobot2_navigation.rviz']
    )
  declare_use_rviz_cmd = DeclareLaunchArgument(
    name='rviz',
    default_value='True',
    description='Whether to start RVIZ')

  declare_rviz_config_path_cmd = DeclareLaunchArgument(
    name='rviz_config_path',
    default_value=default_rviz_config_path,
    description='Full path to the RVIZ config file to use')

  # Nav2
  nav2_launch_path = PathJoinSubstitution(
        [FindPackageShare('nav2_bringup'), 'launch', 'bringup_launch.py']
    )
  default_nav2_config_path = PathJoinSubstitution(
        [pkg_share2, 'config', 'navigation.yaml']
    )
  
  declare_nav2_config_path_cmd = DeclareLaunchArgument(
    name='nav2_config_path',
    default_value=default_nav2_config_path,
    description='Full path to the ROS2 parameters file to use for all launched nodes')

  declare_slam_cmd = DeclareLaunchArgument(
    name='slam',
    default_value='False',
    description='Whether to run SLAM')

  declare_autostart_cmd = DeclareLaunchArgument(
    name='autostart', 
    default_value='true',
    description='Automatically startup the nav2 stack')
    
  # robot_localization moved to gazebo.launch or bringup.launch
  
  # robot_state_publisher moved to description.launch.py
      
  # Launch navigation to the charging dock
  # start_navigate_to_charging_dock_cmd = Node(
  #   package=package_name,
  #   executable=nav_to_charging_dock_script)   

  # Launch navigation to the charging dock
  # start_map_to_base_link_transform_cmd = Node(
  #   package=package_name,
  #   executable='map_to_base_link_transform.py')

  # Launch RViz
  start_rviz_cmd = Node(
    # condition=IfCondition(PythonExpression(['not ', rviz])),
    condition=IfCondition(LaunchConfiguration('rviz')),
    package='rviz2',
    executable='rviz2',
    name='rviz2',
    output='screen',
    arguments=['-d', LaunchConfiguration('rviz_config_path')],
    parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}])

  # Launch the ROS 2 Navigation Stack
  start_ros2_navigation_cmd = IncludeLaunchDescription(
    PythonLaunchDescriptionSource(nav2_launch_path),
    launch_arguments = {'namespace': LaunchConfiguration('namespace'),
                        'use_namespace': LaunchConfiguration('use_namespace'),
                        'slam': LaunchConfiguration('slam'),
                        'map': LaunchConfiguration('map'),
                        'use_sim_time': LaunchConfiguration('use_sim_time'),
                        'params_file': LaunchConfiguration('nav2_config_path'),
                        'autostart': LaunchConfiguration('autostart')}.items())

  # Create the launch description and populate
  ld = LaunchDescription()

  # Declare the launch options
  ld.add_action(declare_namespace_cmd)
  ld.add_action(declare_use_namespace_cmd)
  ld.add_action(declare_autostart_cmd)
  ld.add_action(declare_map_yaml_cmd)
  ld.add_action(declare_nav2_config_path_cmd)
  ld.add_action(declare_rviz_config_path_cmd)
  ld.add_action(declare_slam_cmd)
  ld.add_action(declare_use_rviz_cmd) 
  ld.add_action(declare_use_sim_time_cmd)
  
  # Add any actions

  ld.add_action(start_rviz_cmd)
  # ld.add_action(start_navigate_to_charging_dock_cmd)
  # ld.add_action(start_map_to_base_link_transform_cmd)
  ld.add_action(start_ros2_navigation_cmd)

  return ld
