import launch
import launch_ros.actions
from launch.actions import TimerAction, ExecuteProcess

def generate_launch_description():
    return launch.LaunchDescription([
        # Start turtlesim
        launch_ros.actions.Node(
            package='turtlesim',
            executable='turtlesim_node',
            name='turtlesim'
        ),

        # Delay spawn service call to make sure turtlesim is running first
        TimerAction(
            period=2.0,  # Wait 2 seconds to ensure turtlesim is ready
            actions=[
                ExecuteProcess(
                    cmd=[
                        'ros2', 'service', 'call', '/spawn',
                        'turtlesim/srv/Spawn',
                        '{"x": 5, "y": 4, "theta": 0.0, "name": "turtle2"}'
                    ],
                    output='screen'
                )
            ]
        ),

        # Start gesture control node
        launch_ros.actions.Node(
            package='GestureControlTurtle',
            executable='Controller',
            name='GestureControlTurtle'
        ),
    ])
