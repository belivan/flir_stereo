from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os
import yaml

def generate_launch_description():
    pkg_dir = get_package_share_directory('flir_ros_sync')
    config_file = os.path.join(pkg_dir, 'config', 'atv_both_camera_params.yaml')
    launch_file = os.path.join(pkg_dir, 'launch', 'yamaha_atv', 'atv_single_cam.launch.py')

    with open(config_file, 'r') as f:
        config = yaml.safe_load(f)

    left_camera = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(launch_file),
        launch_arguments={'camera_side': 'left'}.items()
    )

    right_camera = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(launch_file),
        launch_arguments={'camera_side': 'right'}.items()
    )

    set_sync_mode_node = Node(
        package='flir_ros_sync',
        executable='set_sync_mode',
        name='set_flir_sync_mode',
        parameters=[
                    {
                    'sync_mode': config['flir_ros_sync']['sync_mode']['mode'],
                    'serial_list': [
                        config['flir_ros_sync']['left_camera']['serial_port'],
                        config['flir_ros_sync']['right_camera']['serial_port']
                        ]
                    }
                    ],
        output='screen'
    )

    teensy_serial_publisher = Node(
        package='flir_ros_sync',
        executable='check_teensy_status.py',
        name='teensy_serial_publisher',
        output='screen'
    )

    return LaunchDescription([
        left_camera,
        right_camera,
        set_sync_mode_node,
        teensy_serial_publisher
    ])