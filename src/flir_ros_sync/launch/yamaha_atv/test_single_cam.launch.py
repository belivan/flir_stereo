from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Declare launch arguments
    declare_raw_arg = DeclareLaunchArgument(
        "raw", default_value="true", description="Use raw image format"
    )
    declare_flir_id_arg = DeclareLaunchArgument(
        "flir_id", default_value="34582", description="FLIR camera ID"
    )

    # Get the configurations
    raw = LaunchConfiguration("raw")
    flir_id = LaunchConfiguration("flir_id")

    # Path to the single camera launch file
    atv_single_launch_path = os.path.join(
        get_package_share_directory("flir_ros_sync"), "launch", "yamaha_atv", "atv_single_cam.launch.py"
    )

    # Include the single camera launch file for `thermal_one`
    camera_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(atv_single_launch_path),
        launch_arguments={
            "raw": raw,
            "flir_id": flir_id,
            "camera_name": "thermal_left",
            "frame_rate": "60"
        }.items(),
    )

    # Set sync mode node
    set_sync_mode_node = Node(
        package="flir_ros_sync",
        executable="set_sync_mode",
        name="set_flir_sync_mode",
        output="screen",
        parameters=[
            {"sync_mode": 0},  # 0 disable, 1 master, 2 slave
            {"serial_list": [["flir_boson_serial_", flir_id]]}
        ]
    )

    # Optional: Trigger FFC node
    ffc_trigger_node = Node(
        package="flir_ros_sync",
        executable="trigger_ffc.py",
        name="flir_ffc_trigger",
        output="screen",
        parameters=[
            {"serial_list": [["flir_boson_serial_", flir_id]]}
        ]
    )

    # # Optional: Teensy Status publisher
    # teensy_serial_publisher = Node(
    #     package='flir_ros_sync',
    #     executable='check_teensy_status.py',
    #     name='teensy_serial_publisher',
    #     output='screen'
    # )

    # Return the LaunchDescription with the components
    return LaunchDescription([
        declare_raw_arg,
        declare_flir_id_arg,
        camera_launch,
        set_sync_mode_node,
        ffc_trigger_node,
        # teensy_serial_publisher
    ])
