import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    params_file_path = os.path.join(get_package_share_directory(
        'mocap_px4_relay'), 'config', 'vicon_px4_relay_params.yaml')

    ld = LaunchDescription()

    vicon_px4_relay = Node(
        package='mocap_px4_relay',
        executable='vicon_px4_relay',
        output='screen',
        parameters=[params_file_path],
    )

    ld.add_action(vicon_px4_relay)
    return ld
