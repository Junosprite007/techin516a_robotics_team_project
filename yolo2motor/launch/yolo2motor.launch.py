from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # 1) Include the TurtleBot3 GIX hardware launch
    tb3_gix_bringup_share = get_package_share_directory('turtlebot3_gix_bringup')
    hardware_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([tb3_gix_bringup_share, 'launch', 'hardware.launch.py'])
        )
    )

    # 2) USB camera node with params and remap
    usb_cam = Node(
        package='usb_cam',
        executable='usb_cam_node_exe',
        name='usb_cam',
        output='screen',
        parameters=[{
            'video_device': '/dev/video0',
            'image_width': 640,
            'image_height': 360,
        }],
        remappings=[
            ('/image_raw', '/camera/rgb/image_raw'),
        ],
    )

    # 3) YOLO→motor node (your subscriber/trajectory sender)
    yolo2motor = Node(
        package='yolo2motor',
        executable='yolo2motor',
        name='yolo2motor',
        output='screen',
        # If you later want to override params, add a 'parameters=[{...}]' here.
    )

    ping_pong = Node(
        package='yolo2motor',
        executable='ping_pong',
        name='ping_pong',
        output='screen',
    )

    return LaunchDescription([
        hardware_launch,
        usb_cam,
        # yolo2motor,
        ping_pong,
    ])
