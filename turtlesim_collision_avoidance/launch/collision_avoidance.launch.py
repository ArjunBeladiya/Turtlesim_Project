from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Start turtlesim
        Node(
            package='turtlesim',
            executable='turtlesim_node',
            name='sim'
        ),

        # Spawn second turtle
        TimerAction(
            period=2.0,  # wait a bit for turtlesim to start
            actions=[
                ExecuteProcess(
                    cmd=[
                        'ros2', 'service', 'call',
                        '/spawn', 'turtlesim/srv/Spawn',
                        "{x: 2.0, y: 2.0, theta: 0.0, name: 'turtle2'}"
                    ],
                    output='screen'
                ),
            ]
        ),

        # Clear screen
        TimerAction(
            period=3.0,
            actions=[
                ExecuteProcess(
                    cmd=['ros2', 'service', 'call', '/clear', 'std_srvs/srv/Empty', '{}'],
                    output='screen'
                ),
            ]
        ),

        # Disable pen for turtle1
        TimerAction(
            period=3.5,
            actions=[
                ExecuteProcess(
                    cmd=['ros2', 'service', 'call', '/turtle1/set_pen',
                         'turtlesim/srv/SetPen',
                         "{'r': 0, 'g': 0, 'b': 0, 'width': 1, 'off': 1}"],
                    output='screen'
                ),
            ]
        ),

        # Disable pen for turtle2
        TimerAction(
            period=4.0,
            actions=[
                ExecuteProcess(
                    cmd=['ros2', 'service', 'call', '/turtle2/set_pen',
                         'turtlesim/srv/SetPen',
                         "{'r': 0, 'g': 0, 'b': 0, 'width': 1, 'off': 1}"],
                    output='screen'
                ),
            ]
        ),

        # Start random mover for turtle1
        TimerAction(
            period=5.0,
            actions=[
                Node(
                    package='turtlesim_collision_avoidance',
                    executable='random_mover',
                    name='turtle1_mover',
                    parameters=[{'turtle_name': 'turtle1', 'other_turtle': 'turtle2'}]
                ),
            ]
        ),

        # Start random mover for turtle2
        TimerAction(
            period=5.5,
            actions=[
                Node(
                    package='turtlesim_collision_avoidance',
                    executable='random_mover',
                    name='turtle2_mover',
                    parameters=[{'turtle_name': 'turtle2', 'other_turtle': 'turtle1'}]
                ),
            ]
        ),
    ])
