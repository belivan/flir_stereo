from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction, RegisterEventHandler, EmitEvent, LogInfo
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration 
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.events import Shutdown
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Argument declarations
    declare_raw_arg = DeclareLaunchArgument("raw", default_value="true", description="Use raw image format")
    declare_left_flir_id_arg = DeclareLaunchArgument("left_flir_id", default_value="322011", description="Left FLIR camera ID")
    declare_right_flir_id_arg = DeclareLaunchArgument("right_flir_id", default_value="322008", description="Right FLIR camera ID")
    declare_max_retries_arg = DeclareLaunchArgument("max_retries", default_value="5", description="Maximum number of retry attempts")
    declare_retry_delay_arg = DeclareLaunchArgument("retry_delay", default_value="5", description="Delay between retries in seconds")

    # Get configurations
    raw = LaunchConfiguration("raw")
    left_flir_id = LaunchConfiguration("left_flir_id")
    right_flir_id = LaunchConfiguration("right_flir_id")
    max_retries = LaunchConfiguration("max_retries")
    retry_delay = LaunchConfiguration("retry_delay")

    # Path setup
    flir_ros_sync_path = os.path.join(get_package_share_directory('flir_ros_sync'), 'launch', 'yamaha_atv', 'atv_single_cam.launch.py')

    # Define camera launch function
    def create_camera_launch(camera_id, camera_name, retry_count=0):
        return TimerAction(
            period=retry_delay,
            actions=[
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(flir_ros_sync_path),
                    launch_arguments={
                        'raw': raw,
                        'flir_id': camera_id,
                        'camera_name': camera_name,
                        'frame_rate': '10',
                    }.items()
                )
            ]
        )

    # Create camera launches
    left_camera = create_camera_launch(left_flir_id, 'thermal_left')
    right_camera = create_camera_launch(right_flir_id, 'thermal_right')

    # Error handlers with retry logic
    left_camera_handler = RegisterEventHandler(
        OnProcessExit(
            target_action=left_camera,
            on_exit=[
                LogInfo(msg=lambda ctx: f"Left camera attempt failed. Retry {ctx.locals.left_retries}/{max_retries}"),
                TimerAction(
                    period=retry_delay,
                    actions=[create_camera_launch(left_flir_id, 'thermal_left')],
                    condition=lambda ctx: ctx.locals.left_retries < int(max_retries.perform(ctx))
                ),
                EmitEvent(
                    event=Shutdown(reason='Left camera failed after max retries'),
                    condition=lambda ctx: ctx.locals.left_retries >= int(max_retries.perform(ctx))
                )
            ]
        )
    )

    right_camera_handler = RegisterEventHandler(
        OnProcessExit(
            target_action=right_camera,
            on_exit=[
                LogInfo(msg=lambda ctx: f"Right camera attempt failed. Retry {ctx.locals.right_retries}/{max_retries}"),
                TimerAction(
                    period=retry_delay,
                    actions=[create_camera_launch(right_flir_id, 'thermal_right')],
                    condition=lambda ctx: ctx.locals.right_retries < int(max_retries.perform(ctx))
                ),
                EmitEvent(
                    event=Shutdown(reason='Right camera failed after max retries'),
                    condition=lambda ctx: ctx.locals.right_retries >= int(max_retries.perform(ctx))
                )
            ]
        )
    )

    # Set sync mode node
    set_sync_mode_node = Node(
        package='flir_ros_sync',
        executable='set_sync_mode',
        name='set_flir_sync_mode',
        output='screen',
        parameters=[{
            'sync_mode': 0,  # 0 disable, 1 master, 2 slave
            'serial_list': [
                ["flir_boson_serial_", left_flir_id],
                ["flir_boson_serial_", right_flir_id]
            ]
        }]
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
        declare_max_retries_arg,
        declare_retry_delay_arg,
        left_camera,
        right_camera,
        set_sync_mode_node,
        teensy_serial_publisher,
        left_camera_handler,
        right_camera_handler
    ])