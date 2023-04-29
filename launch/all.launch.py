#!/usr/bin/python3
# -*- coding: utf-8 -*-
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription, ExecuteProcess, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')
    pkg_sim_car = get_package_share_directory('rtec_vehicle_sim')
    #launch_dir = os.path.join(pkg_sim_car, 'launch')
    #rviz_config_path = os.path.join(pkg_sim_car, 'config', 'rtec_sim.rviz')
    # pkg_joy = get_package_share_directory('joy')

    # Gazebo launch
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gazebo.launch.py'),
        )
    )
    
    # Spawn a vehicle
    car = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_sim_car, 'launch', 'spawn_car.launch.py'),
        )
    )

    # rviz2 launch
    rviz = Node(package='rviz2', 
                namespace='', 
                executable='rviz2', 
                name='rviz2',
                arguments=['-d', [os.path.join(pkg_sim_car, 'config', 'rtec_sim.rviz')]])

    tf_odom_base = Node(package = "tf2_ros", 
                       executable = "static_transform_publisher",
                       arguments = ["0", "0", "0", "0", "0", "0", "odom", "base_footprint"])
    
    #tf_map_odom = Node(package = "tf2_ros", 
    #                   executable = "static_transform_publisher",
    #                   arguments = ["0", "0", "0", "0", "0", "0", "map", "odom"])
    
    #joy = Node(
    #        package='joy', executable='joy_node', output='screen');
    #teleop_twist = Node(
    #        package='teleop_twist_joy', executable='teleop_node', output='screen', parameters=[os.path.join(pkg_sim_car, "config", "teleop_twist_joy.yaml")])

    # launches
    return LaunchDescription([
        DeclareLaunchArgument(
          'world',
          default_value=[os.path.join(pkg_sim_car, 'worlds', 'simple.world'), ''],
          description='SDF world file'),
        gazebo,
        car,
        #tf_odom_base,
        #tf_map_odom,
        TimerAction(
            period=5.0,
            actions=[rviz],
        )
        #joy,
        #teleop_twist
    ])
