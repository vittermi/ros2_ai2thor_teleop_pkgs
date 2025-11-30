from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Static TF: base_link → camera_link (z = 0.675 m) from ai2thor docs
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_to_camera_tf',
            arguments=['0', '0', '0.675', '0', '0', '0', 'base_link', 'camera_link'],
        ),

        Node(
            package='ai2thor_bridge',
            executable='odom_adapter',
            name='odom_adapter',
            output='screen',
        ),

        Node(
            package='ai2thor_bridge',
            executable='depth_image_adapter',
            name='depth_image_adapter',
            output='screen',
        ),

        Node(
            package='depthimage_to_laserscan',
            executable='depthimage_to_laserscan_node',
            name='depth_to_scan',
            output='screen',
            parameters=[{
                'output_frame': 'camera_link',
                'range_min': 0.3,
                'range_max': 5.0,
                'scan_time': 0.033,
                'scan_height': 10,
            }],
            remappings=[
                ('depth', '/depth/image_raw'),
                ('scan', '/scan')
            ]
        ),
    ])
