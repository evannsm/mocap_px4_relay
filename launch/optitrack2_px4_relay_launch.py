import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    params_file_path = os.path.join(get_package_share_directory(
        'mocap_px4_relay'), 'config', 'optitrack2_px4_relay_params.yaml')

    ld = LaunchDescription()

    optitrack2_px4_relay = Node(
        package='mocap_px4_relay',
        executable='optitrack_px4_relay',
        output='screen',
        parameters=[params_file_path],
    )

    full_state_relay = Node(
        package='mocap_px4_relay',
        executable='full_state_relay',
        output='screen',
        parameters=[params_file_path],
    )


    ld.add_action(optitrack2_px4_relay)
    ld.add_action(full_state_relay)
    return ld
