import os

from ament_index_python.packages import get_package_share_path

from launch import LaunchDescription
from launch_ros.actions import Node

from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition

import xacro

pkg_folder = 'modelrobot3r_pkg_ga'
robot_file = 'robot_3r_ga.urdf.xacro'

rviz_file = 'rviz3r_config.rviz'

def generate_launch_description():

    # Check if we're told to use sim time
    use_sim_time = LaunchConfiguration('use_sim_time')

    pkg_path = os.path.join(get_package_share_path(pkg_folder))
    default_model_path = os.path.join(pkg_path + '/model/' + robot_file)
    default_rviz_config_path = os.path.join(pkg_path + '/model/' + rviz_file)

    rviz_arg = DeclareLaunchArgument(name='rvizconfig', default_value=str(default_rviz_config_path), description='Absolute path to rviz config file')

    # Process the URDF file
    robot_description_config = xacro.process_file(default_model_path)

    params = {'robot_description': robot_description_config.toxml()}

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[params]
    )

    # Nodo para el controlador de la rueda
    wheel_controller_node = Node(
        package=pkg_folder,  # Cambia al nombre correcto del paquete donde esté tu nodo
        executable='WheelController',  # Cambia al nombre del ejecutable de tu nodo
        output='screen',
    )

    # Depending on gui parameter, either launch joint_state_publisher or joint_state_publisher_gui
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', LaunchConfiguration('rvizconfig')],
        parameters=[{'use_sim_time': True}]  # Ensure time sync (if needed)
    )

    # Launch!
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use sim time if true'),
        robot_state_publisher_node,
        wheel_controller_node,  # Agrega el nodo controlador de la rueda
        rviz_arg,
        rviz_node,
    ])