from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

def generate_launch_description():
    # Declare arguments
    declare_raw_arg = DeclareLaunchArgument("raw", default_value="true", description="Use raw image format")
    declare_flir_id_arg = DeclareLaunchArgument("flir_id", default_value="322008", description="FLIR camera ID")
    declare_camera_name_arg = DeclareLaunchArgument("camera_name", default_value="thermal_left", description="Camera name")
    declare_timestamp_offset_arg = DeclareLaunchArgument("timestamp_offset", default_value="0.0", description="Timestamp offset")
    declare_frame_rate_arg = DeclareLaunchArgument("frame_rate", default_value="10", description="Frame rate")
    declare_ffc_interval_arg = DeclareLaunchArgument("ffc_interval", default_value="3", description="Interval between FFC in minutes")

    # Retrieve arguments
    raw = LaunchConfiguration("raw")
    flir_id = LaunchConfiguration("flir_id")
    camera_name = LaunchConfiguration("camera_name")
    timestamp_offset = LaunchConfiguration("timestamp_offset")
    frame_rate = LaunchConfiguration("frame_rate")
    ffc_interval = LaunchConfiguration("ffc_interval")

    # Define container for composable nodes
    container = ComposableNodeContainer(
        name=["flir_nodelet_manager_", flir_id],
        namespace='',
        package='rclcpp_components',
        executable='component_container_mt',
        output='screen',
        composable_node_descriptions=[
            ComposableNode(
                package='flir_ros_sync',
                plugin='flir_ros_sync::FlirRos',
                name=["flir_nodelet_", flir_id],
                parameters=[
                    {"device_name": ["/dev/flir_boson_video_", flir_id]},
                    {"serial_port": ["/dev/flir_boson_serial_", flir_id]},
                    {"raw": raw},
                    {"publish_image_sharing_every_n": 1},
                    {"send_every_n": 1},
                    {"timestamp_offset": timestamp_offset},
                    {"use_ext_sync": 1},
                    {"width": 640},
                    {"height": 512},
                    {"camera_name": camera_name},
                    {"intrinsic_url": [
                        "package://flir_ros_sync/data/camera_info/yamaha_atv/", camera_name, ".yaml"
                    ]},
                    {"gain_mode": 2},  # 0 high, 1 low, 2 auto, 3 dual, 4 manual
                    {"ffc_mode": 0},  # 0 manual, 1 auto, 2 external, 3 shutter test
                    {"frame_rate": frame_rate},
                    {"ffc_interval": ffc_interval}, # Interval between FFC in minutes
                ],
                remappings=[
                    ("/set_camera_info", [camera_name, "/set_camera_info"])
                ],
            ),
        ]
    )

    return LaunchDescription([
        declare_raw_arg,
        declare_flir_id_arg,
        declare_camera_name_arg,
        declare_timestamp_offset_arg,
        declare_frame_rate_arg,
        declare_ffc_interval_arg,
        container
    ])
