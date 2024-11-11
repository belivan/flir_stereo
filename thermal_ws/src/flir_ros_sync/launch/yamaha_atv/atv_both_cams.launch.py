from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Declare arguments
    declare_raw_arg = DeclareLaunchArgument("raw", default_value="true", description="Use raw image format")
    declare_left_flir_id_arg = DeclareLaunchArgument("left_flir_id", default_value="322011", description="Left FLIR camera ID")
    declare_right_flir_id_arg = DeclareLaunchArgument("right_flir_id", default_value="322008", description="Right FLIR camera ID")

    # Get the launch configurations
    raw = LaunchConfiguration("raw")
    left_flir_id = LaunchConfiguration("left_flir_id")
    right_flir_id = LaunchConfiguration("right_flir_id")

    # Paths to sub-launch file (atv-single.launch)
    flir_ros_sync_path = os.path.join(get_package_share_directory('flir_ros_sync'), 'launch', 'yamaha_atv', 'atv_single_cam.launch.py')

    # Include left and right camera launch files
    left_camera = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(flir_ros_sync_path),
        launch_arguments={
            'raw': raw,
            'flir_id': left_flir_id,
            'camera_name': 'thermal_left',
            'frame_rate': '10'
        }.items(),
    )

    right_camera = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(flir_ros_sync_path),
        launch_arguments={
            'raw': raw,
            'flir_id': right_flir_id,
            'camera_name': 'thermal_right',
            'frame_rate': '10'
        }.items(),
    )

    # Set sync mode node
    set_sync_mode_node = Node(
        package='flir_ros_sync',
        executable='set_sync_mode',
        name='set_flir_sync_mode',
        output='screen',
        parameters=[
            {
                'sync_mode': 2,
                'serial_list': [
                    ["flir_boson_serial_", left_flir_id],
                    ["flir_boson_serial_", right_flir_id]
                ]
            }
        ]
    )

    # Trigger FFC node
    ffc_trigger_node = Node(
        package='flir_ros_sync',
        executable='trigger_ffc.py',
        name='flir_ffc_trigger',
        output='screen',
        parameters=[
            {
                'serial_list': [
                    ["flir_boson_serial_", left_flir_id],
                    ["flir_boson_serial_", right_flir_id]
                ]
            }
        ]
    )

    # Teensy Status publisher
    teensy_serial_publisher = Node(
        package='flir_ros_sync',
        executable='check_teensy_status.py',
        name='teensy_serial_publisher',
        output='screen'
    )

    return LaunchDescription([
        declare_raw_arg,
        declare_left_flir_id_arg,
        declare_right_flir_id_arg,
        left_camera,
        right_camera,
        set_sync_mode_node,
        ffc_trigger_node,
        teensy_serial_publisher
    ])
