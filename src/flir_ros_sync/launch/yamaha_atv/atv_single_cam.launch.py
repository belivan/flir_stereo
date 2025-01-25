from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from ament_index_python.packages import get_package_share_directory
import yaml, os

def generate_launch_description():
    # Declare the argument
    camera_arg = DeclareLaunchArgument('camera_side', default_value='left')
    camera_side = LaunchConfiguration('camera_side')
    
    # Load YAML once at parse-time
    pkg_dir = get_package_share_directory('flir_ros_sync')
    config_file = os.path.join(pkg_dir, 'config', 'atv_both_camera_params.yaml')
    with open(config_file, 'r') as f:
        config = yaml.safe_load(f)

    def launch_setup(context, *args, **kwargs):
        # 1. Resolve the 'camera_side' at runtime
        side_str = camera_side.perform(context)  # 'left' or 'right'
        
        # 2. Pick the correct sub-dict from config
        params = config['flir_ros_sync'][f"{side_str}_camera"]

        # 3. Construct your node(s) with those params
        node = ComposableNode(
            package='flir_ros_sync',
            plugin='flir_ros_sync::FlirRos',
            name=[f"flir_nodelet_{side_str}"],
            parameters=[params]
        )

        container = ComposableNodeContainer(
            name=[f"flir_nodelet_manager_{side_str}"],
            namespace='',
            package='rclcpp_components',
            executable='component_container_mt',
            composable_node_descriptions=[node],
            output='screen'
        )
        # Return a list of actions to launch
        return [container]

    return LaunchDescription([
        camera_arg,
        OpaqueFunction(function=launch_setup)
    ])
