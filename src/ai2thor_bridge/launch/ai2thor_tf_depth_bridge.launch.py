from launch import LaunchDescription
from launch_ros.actions import Node # type: ignore
from launch_ros.actions import LifecycleNode

from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution

def generate_launch_description():
    
    nav2_params = PathJoinSubstitution([
        FindPackageShare('ai2thor_bridge'),
        'config',
        'nav2_params.yaml'
    ])

    bt_navigator = LifecycleNode(
        package='nav2_bt_navigator',
        executable='bt_navigator',
        name='bt_navigator',
        namespace='',
        output='screen',
        parameters=[nav2_params],
    )

    planner_server = LifecycleNode(
        package='nav2_planner',
        executable='planner_server',
        name='planner_server',
        namespace='',
        output='screen',
        parameters=[nav2_params],
    )

    controller_server = LifecycleNode(
        package='nav2_controller',
        executable='controller_server',
        name='controller_server',
        namespace='',
        output='screen',
        parameters=[nav2_params],
    )

    behavior_server = LifecycleNode(
        package='nav2_behaviors',
        executable='behavior_server',
        name='behavior_server',
        namespace='',
        output='screen',
        parameters=[nav2_params],
    )

    waypoint_follower = LifecycleNode(
        package='nav2_waypoint_follower',
        executable='waypoint_follower',
        name='waypoint_follower',
        namespace='',
        output='screen',
        parameters=[nav2_params],
    )

    lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_navigation',
        output='screen',
        parameters=[nav2_params],
    )


    return LaunchDescription([
        # Static TF: base_link → camera_link (z = 0.675 m) from ai2thor docs
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_to_camera_tf',
            arguments=[
                '0', '0', '1.6',
                '0', '0', '0',
                'base_footprint', 'camera_link'
            ],
        ),


        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='camera_optical_tf',
            arguments=[
                '0', '0', '0',
                '-1.5708', '0', '-1.5708',   # roll/pitch/yaw old-style: yaw, pitch, roll
                'camera_link', 'camera_depth_optical_frame'
            ],
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
            package='ai2thor_bridge',
            executable='cmd_vel_adapter',
            name='cmd_vel_adapter',
            output='screen',
        ),

        Node(
            package='rtabmap_slam',
            executable='rtabmap',
            name='rtabmap',
            output='screen',
            parameters=[{
                'frame_id': 'base_footprint',        
                'odom_frame_id': 'odom',
                'map_frame_id': 'map',
                'publish_tf': True,            

                'subscribe_depth': True,
                'subscribe_rgb': True,        
                'subscribe_rgbd': False,
                'subscribe_scan': False,       
                'subscribe_scan_cloud': False,
                'subscribe_odom_info': False,
                'subscribe_odom': True,
                'subscribe_imu': False,

                'approx_sync': True,
                'sync_queue_size': 30,
                'topic_queue_size': 30,

                'RGBD/CreateOccupancyGrid': 'true',
                'Grid/FromDepth': 'true',
                'Grid/RangeMax': '10.0',     

                'Grid/3D': 'false',

                'Grid/NormalsSegmentation': 'false',
                'Grid/MinGroundHeight': '-0.10',   
                'Grid/MaxGroundHeight': '0.10',  

                'Grid/MaxObstacleHeight': '1.5',   

                'Grid/CellSize': '0.1',          
                'Grid/NoiseFilteringRadius': '0.2',
                'Grid/NoiseFilteringMinNeighbors': '2',

                'Rtabmap/DetectionRate': '5.0',

                'map_always_update': True,
            }],
            remappings=[
                ('depth/image', '/depth/image_raw'),
                ('/odom_info', '/odom'),
                ('/rgb/camera_info', '/depth/camera_info')
            ]
        ),

        Node(
            package='depthimage_to_laserscan',
            executable='depthimage_to_laserscan_node',
            name='depth_to_scan',
            output='screen',
            parameters=[{
                'output_frame': 'camera_link',
                'range_min': 0.1,
                'range_max': 1.0,
                'scan_time': 0.033,
                'scan_height': 50,
            }],
            remappings=[
                ('depth', '/depth/image_raw'),
                ('depth_camera_info', '/depth/camera_info'),
                ('scan', '/scan')
            ]
        ),

        controller_server,
        planner_server,
        bt_navigator,
        behavior_server,
        waypoint_follower,
        lifecycle_manager,
    ])
