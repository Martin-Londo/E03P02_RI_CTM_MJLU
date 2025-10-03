import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.conditions import IfCondition, UnlessCondition

import xacro


def generate_launch_description():

    # Check if we're told to use sim time
    use_sim_time = LaunchConfiguration('use_sim_time')

    # Process the URDF file
    pkg_path = os.path.join(get_package_share_directory('SCARA_tray_planner'))
    xacro_file = os.path.join(pkg_path,'models','scara_digital_twin.urdf.xacro')
    config_file = os.path.join(pkg_path,'models','rviz_config.rviz')
    robot_description_config = xacro.process_file(xacro_file)

    # Create a robot_state_publisher node
    params = {'robot_description': robot_description_config.toxml(), 'use_sim_time': use_sim_time}

    dxf_parser_node = Node(
        package='SCARA_tray_planner',
        executable='dxf_parser_node2',
        output='screen',
        parameters=[{'use_sim_time': True}]  # Ensure time sync (if needed)
    )

    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[params]
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        parameters=[{'use_sim_time': True}],  # Ensure time sync (if needed)
        arguments=['-d', config_file]
    ) 

    # Launch!
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use sim time if true'),
        node_robot_state_publisher,
        rviz_node,
        dxf_parser_node
    ])