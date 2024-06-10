from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration, EnvironmentVariable

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'port',
            default_value='9090',
            description='Port to use for the rosbridge server'
        ),
        DeclareLaunchArgument(
            'websocket_external_port',
            default_value=EnvironmentVariable('SLOT_ROSBRIDGE_PORT', default_value='9090'),
            description='External port for the rosbridge websocket'
        ),
        Node(
            package='rosbridge_server',
            executable='rosbridge_websocket',
            name='rosbridge_websocket',
            output='screen',
            parameters=[{'port': LaunchConfiguration('port')}],
            arguments=['--external_port', LaunchConfiguration('websocket_external_port')]
        ),
        Node(
            package='web_video_server',
            executable='web_video_server',
            name='web_video_server',
            output='screen',
            parameters=[{'port': 11315}]
        )
    ])
