from launch import LaunchDescription
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os
import yaml

def generate_launch_description():
    pkg_dir = get_package_share_directory('flir_ros_sync')
    config_file = os.path.join(pkg_dir, 'config', 'atv_both_camera_params.yaml')
    
    with open(config_file, 'r') as f:
        config = yaml.safe_load(f)

    # Declare arguments
    declare_camera_arg = DeclareLaunchArgument(
        "camera_side", 
        default_value="left",
        description="Camera side (left/right)"
    )
    camera_side = LaunchConfiguration("camera_side")

    container = ComposableNodeContainer(
        name=f"flir_nodelet_manager_{camera_side}",
        namespace='',
        package='rclcpp_components',
        executable='component_container_mt',
        output='screen',
        composable_node_descriptions=[
            ComposableNode(
                package='flir_ros_sync',
                plugin='flir_ros_sync::FlirRos',
                name=f"flir_nodelet_{camera_side}",
                parameters=[config['flir_ros_sync'][f"{camera_side}_camera"]],
            ),
        ]
    )

    return LaunchDescription([
        declare_camera_arg,
        container
    ])