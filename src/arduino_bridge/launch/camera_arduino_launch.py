from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    video_src = LaunchConfiguration('video_source')
    video_width = LaunchConfiguration('video_width')
    video_height = LaunchConfiguration('video_height')
    video_fps = LaunchConfiguration('video_fps')
    serial_port = LaunchConfiguration('serial_port')
    publish_rate = LaunchConfiguration('publish_rate')
    min_area = LaunchConfiguration('min_area')
    show_window = LaunchConfiguration('show_window')

    return LaunchDescription([
        DeclareLaunchArgument('video_source', default_value='0', description='Camera device or pipeline'),
        DeclareLaunchArgument('video_width', default_value='640', description='Camera capture width'),
        DeclareLaunchArgument('video_height', default_value='480', description='Camera capture height'),
        DeclareLaunchArgument('video_fps', default_value='30', description='Camera capture framerate'),
        DeclareLaunchArgument('serial_port', default_value='/dev/ttyACM0', description='Serial port to Arduino'),
        DeclareLaunchArgument('publish_rate', default_value='5.0', description='Camera publish rate'),
        DeclareLaunchArgument('min_area', default_value='2000', description='Minimum mask area'),
        DeclareLaunchArgument('show_window', default_value='false', description='Show debug window'),

        Node(
            package='arduino_bridge',
            executable='camera_detect',
            name='camera_detect',
            output='screen',
            parameters=[{
                'video_source': video_src,
                'video_width': video_width,
                'video_height': video_height,
                'video_fps': video_fps,
                'publish_rate': publish_rate,
                'min_area': min_area,
                'show_window': show_window,
            }],
        ),

        Node(
            package='arduino_bridge',
            executable='arduino_serial',
            name='arduino_serial',
            output='screen',
            parameters=[{'serial_port': serial_port}],
        ),
    ])
