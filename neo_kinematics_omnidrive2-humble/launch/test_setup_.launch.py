import launch
import launch.actions
import launch.substitutions
import os
from ament_index_python.packages import get_package_share_directory
import launch_ros.actions

def generate_launch_description():
    config = os.path.join(get_package_share_directory('neo_kinematics_omnidrive2'),'launch','test_setup.yaml')
    config1 = os.path.join(get_package_share_directory('neo_kinematics_omnidrive2'),'launch','test_setup_socket.yaml')

    return launch.LaunchDescription([
        launch_ros.actions.Node(
            package='neo_kinematics_omnidrive2', executable='neo_omnidrive_node', output='screen',
            name='neo_omnidrive_node', parameters = [config]), 
        launch_ros.actions.Node(
            package='neo_kinematics_omnidrive2', executable='neo_omnidrive_socketcan_node', output='screen',
            name='neo_omnidrive_socketcan_node', parameters = [config1])
    ])