from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    topic = DeclareLaunchArgument('topic', default_value='/ai2thor/teleop_cmd')
    step_m = DeclareLaunchArgument('step_m', default_value='0.25')        # meters per step
    step_deg = DeclareLaunchArgument('step_deg', default_value='10.0')     # degrees per step
    repeat_hz = DeclareLaunchArgument('repeat_hz', default_value='10.0')   # key-hold repeat

    return LaunchDescription([
        topic, step_m, step_deg, repeat_hz,
        Node(
            package='ai2thor_teleop_cli',
            executable='teleop_key',
            name='ai2thor_teleop_key',
            output='screen',
            parameters=[{
                'topic': LaunchConfiguration('topic'),
                'step_m': LaunchConfiguration('step_m'),
                'step_deg': LaunchConfiguration('step_deg'),
                'repeat_hz': LaunchConfiguration('repeat_hz'),
            }]
        )
    ])
