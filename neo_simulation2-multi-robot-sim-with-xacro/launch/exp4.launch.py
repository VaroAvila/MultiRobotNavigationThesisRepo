import launch
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os
import time

MY_NO_ROBOTS = "3"
MY_NEO_ENVIRONMENT = "map_obstacles"

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

    ld.add_action(TimerAction(
        period=3.0,
        actions=[IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory('neo_simulation2'), 'launch', 'simulation.launch.py')
            ),
            launch_arguments={
                'use_multi_robots': 'True',
                'x': str(3.7),
                'y': str(0.3),
                'yaw': str(3.14),
                'namespace_robot': "robot0"
            }.items(),
        )]
    ))

    ld.add_action(TimerAction(
        period=4.0,
        actions=[IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory('neo_simulation2'), 'launch', 'simulation.launch.py')
            ),
            launch_arguments={
                'use_multi_robots': 'True',
                'x': str(-2.85),
                'y': str(0.6),
                'yaw': str(0),
                'namespace_robot': "robot1"
            }.items(),
        )]
    ))

    ld.add_action(TimerAction(
        period=5.0,
        actions=[IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory('neo_simulation2'), 'launch', 'simulation.launch.py')
            ),
            launch_arguments={
                'use_multi_robots': 'True',
                'x': str(3.5),
                'y': str(3.2),
                'yaw': str(3.14),
                'namespace_robot': "robot2"
            }.items(),
        )]
    ))

    return ld
