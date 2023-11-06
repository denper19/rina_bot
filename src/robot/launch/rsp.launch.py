import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration, Command
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node

import xacro


def generate_launch_description():

    # Check if we're told to use sim time
    use_sim_time = LaunchConfiguration('use_sim_time')
    use_ros2_control = LaunchConfiguration('use_ros2_control')
    # use_ros2_control = LaunchConfiguration('use_rviz')

    # Process the URDF file
    pkg_path = os.path.join(get_package_share_directory('robot'))
    xacro_file = os.path.join(pkg_path,'urdf','robot_core.urdf.xacro')
    rviz_file = os.path.join(pkg_path,'config','viz.rviz')

    # robot_description_config = xacro.process_file(xacro_file).toxml()
    robot_description_config = Command(['xacro ', xacro_file, ' use_ros2_control:=', use_ros2_control, ' sim_mode:=', use_sim_time])
    
    # Create a robot_state_publisher node
    params = {'robot_description': robot_description_config, 'use_sim_time': use_sim_time}
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[params]
    )

    # rviz2_node = Node(package='rviz2',
    #             node_executable='rviz2',
    #             node_name='rviz2',
    #             arguments=['-d', rviz_file],
    #             )

    # Launch!
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use sim time if true'),
        DeclareLaunchArgument(
            'use_ros2_control',
            default_value='true',
            description='Use ros2 control'),
        # DeclareLaunchArgument(
        #     'use_rviz',
        #     default_value='true',
        #     description='Provide rviz display'),
        node_robot_state_publisher,
        # rviz2_node
    ])