import launch
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('rviz', default_value='true', description='Launch RViz?'),
        DeclareLaunchArgument('show_filtered_image', default_value='true', description='Show the filtered image stream?'),

        Node(
            package='kinect_ros2', # Replace with the actual package name
            executable='kinect_color_filter_node', # Name of your depth calculator node executable
            name='kinect_color_filter_node',
            parameters=[{"window_name": "Filtered Image"}],
            remappings=[("image", "kinect/image_raw")],
        ),

        Node(
            package='kinect_ros2', # Replace with the actual package name
            executable='kinect_shape_filter_node', # Name of your filtered image viewer node executable
            name='kinect_shape_filter_node',
            condition=launch.conditions.IfCondition(LaunchConfiguration('show_filtered_image'))
        ),

        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            condition=launch.conditions.IfCondition(LaunchConfiguration('rviz'))
        ),
        
        LogInfo(
            condition=launch.conditions.IfCondition(LaunchConfiguration('show_filtered_image')),
            msg="Filtered image viewer node is running. Open RViz to view the filtered image stream."
        ),
    ])
