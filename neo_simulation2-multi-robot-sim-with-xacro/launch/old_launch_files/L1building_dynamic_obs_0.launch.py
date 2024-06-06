# Neobotix GmbH

import launch
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import ThisLaunchFileDir, LaunchConfiguration
from launch_ros.actions import Node
from launch.launch_context import LaunchContext
from launch.utilities import perform_substitutions
import os
from pathlib import Path
import xml.etree.ElementTree as ET
import xacro
import time

MY_NO_ROBOTS = "3"
MY_NEO_ENVIRONMENT = "L1building_dynamic_obs_0"

def generate_launch_description():
    default_world_path = os.path.join(get_package_share_directory('neo_simulation2'), 'worlds', MY_NEO_ENVIRONMENT + '.world')

    global MY_NO_ROBOTS 
    if(int(MY_NO_ROBOTS) > 5):
        print("Warn: Having more than 5 robots is not a good idea - too much overhead")
        print("Therefore Let's spawn 5")
        MY_NO_ROBOTS = '5'
        time.sleep(5)   

    ld = LaunchDescription()
    gazebo = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')
            ),
            launch_arguments={
                'world': default_world_path,
                'verbose': 'true',
            }.items()
        )
    ld.add_action(gazebo)

    ld.add_action(IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('neo_simulation2'), 'launch', 'simulation_seats_robot.launch.py')
        ),
        launch_arguments={
            'use_multi_robots': 'True',
            'x': str(-25.5),
            'y': str(0),
            'yaw': str(0),
            'namespace_robot': "robot0"
        }.items(),
    ))

    ld.add_action(IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('neo_simulation2'), 'launch', 'simulation_locker_robot.launch.py')
        ),
        launch_arguments={
            'use_multi_robots': 'True',
            'x': str(-25.5),
            'y': str(-2),
            'yaw': str(0),
            'namespace_robot': "robot1"
        }.items(),
    ))

    # ld.add_action(IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(
    #         os.path.join(get_package_share_directory('neo_simulation2'), 'launch', 'simulation_frame_robot.launch.py')
    #     ),
    #     launch_arguments={
    #         'use_multi_robots': 'True',
    #         'x': str(-25.5),
    #         'y': str(-4),
    #         'yaw': str(0),
    #         'namespace_robot': "robot2"
    #     }.items(),
    # ))

    return ld