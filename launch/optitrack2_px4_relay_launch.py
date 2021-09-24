import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    params_file_path = os.path.join(get_package_share_directory(
        'optitrack2_px4_relay'), 'config', 'optitrack2_px4_relay_params.yaml')

    ld = LaunchDescription()

    optitrack2_px4_relay = Node(
        package='optitrack2_px4_relay',
        executable='optitrack2_px4_relay',
        output='screen',
        parameters=[params_file_path],
    )

    ld.add_action(optitrack2_px4_relay)
    return ld
