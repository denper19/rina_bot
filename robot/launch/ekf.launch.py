# Author: Addison Sears-Collins
# Date: August 31, 2021
# Description: Launch a basic mobile robot
# https://automaticaddison.com

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():

  # Set the path to different files and folders.
  robot_localization_file_path = os.path.join(get_package_share_directory('robot'), 'config/ekf.yaml') 
  
  use_sim_time = LaunchConfiguration('use_sim_time')

  declare_use_sim_time = DeclareLaunchArgument(
    name='use_sim_time',
    default_value='True',
    description='Use simulation (Gazebo) clock if true')

  # Start robot localization using an Extended Kalman filter
  robot_localization_node = Node(
    package='robot_localization',
    executable='ekf_node',
    name='ekf_filter_node',
    output='screen',
    parameters=[robot_localization_file_path, {'use_sim_time': use_sim_time}]
    )

  # Create the launch description and populate
  return LaunchDescription([
    declare_use_sim_time,
    robot_localization_node,
  ])