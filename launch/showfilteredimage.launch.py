import launch
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([

        # Node(
        #     package="kinect_ros2",
        #     executable="kinect_ros2_node",
        #     name="kinect_ros2",
        #     namespace="kinect"
        # ),
       
        # Node(
        #     package="kinect_ros2",
        #     executable="kinect_color_filter_node",
        #     name="kinect_ros2",
        #     namespace="kinect1"
        # ),

        # Node(
        #     package="kinect_ros2",
        #     executable="kinect_shape_filter_node",
        #     name="kinect_ros2",
        #     namespace="kinect2"
        # ),

        Node(
            package="image_tools",
            executable="showimage",
            name="rgb_showimage",
            parameters=[{"window_name": "Filtered RGB"}],
            remappings=[("image", "/filtered_rgb")],
        ),

        Node(
            package="image_tools",
            executable="showimage",
            name="circles_showimage",
            parameters=[{"window_name": "Filtered Circles"}],
            remappings=[("image", "/filtered_shapes")],
        ),

        Node(
            package="image_tools",
            executable="showimage",
            name="circles_showimage",
            parameters=[{"window_name": "Raw Feed"}],
            remappings=[("image", "/image_raw")],
        ),


    ])
