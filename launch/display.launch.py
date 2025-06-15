#!/usr/bin/python3
# -*- coding: utf-8 -*-
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_prefix

def generate_launch_description():
    pkg_ign_gazebo = get_package_share_directory('ros_ign_gazebo')
    pkg_eklavya_bot = get_package_share_directory('ros_task_eklavya')

    description_package_name = "ros_task_eklavya"
    install_dir = get_package_prefix(description_package_name)
    gazebo_models_path = os.path.join(pkg_eklavya_bot, 'models')

    if 'IGN_GAZEBO_RESOURCE_PATH' in os.environ:
        os.environ['IGN_GAZEBO_RESOURCE_PATH'] = os.environ['IGN_GAZEBO_RESOURCE_PATH'] + ':' + install_dir + '/share' + ':' + gazebo_models_path
    else:
        os.environ['IGN_GAZEBO_RESOURCE_PATH'] = install_dir + '/share' + ':' + gazebo_models_path


     # Launch Ignition Gazebo
    ign_gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ign_gazebo, 'launch', 'ign_gazebo.launch.py'),
        ),
        launch_arguments={'ign_args': '-r empty.sdf'}.items()
    )

     # Include spawn launch
    spawn_bot = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_eklavya_bot, 'launch', 'spawn_robot.launch.py'),
        )
    )


    return LaunchDescription([
        ign_gazebo,
        spawn_bot
    ])



    # INSTRUCTIONS:
    # 1. Fill in the missing code above to launch Ignition Gazebo and your robot spawner.
    # 2. Make sure to use the correct package names and file paths.
    # 3. Set up the environment variable as shown in the solution.
