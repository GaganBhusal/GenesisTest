from launch import LaunchDescription
from launch_ros.actions import Node

PARAMS = '/home/yayy/My/Codeeeeee/Simulators/GenesisWorkingMine/nav2_params.yaml'

def generate_launch_description():
    return LaunchDescription([

        Node(package='nav2_controller',      executable='controller_server',  name='controller_server',  parameters=[PARAMS], output='screen'),
        Node(package='nav2_smoother',        executable='smoother_server',    name='smoother_server',    parameters=[PARAMS], output='screen'),
        Node(package='nav2_planner',         executable='planner_server',     name='planner_server',     parameters=[PARAMS], output='screen'),
        Node(package='nav2_behaviors',       executable='behavior_server',    name='behavior_server',    parameters=[PARAMS], output='screen'),
        Node(package='nav2_bt_navigator',    executable='bt_navigator',       name='bt_navigator',       parameters=[PARAMS], output='screen'),
        Node(package='nav2_waypoint_follower', executable='waypoint_follower', name='waypoint_follower', parameters=[PARAMS], output='screen'),
        Node(package='nav2_velocity_smoother', executable='velocity_smoother', name='velocity_smoother', parameters=[PARAMS], output='screen'),
        Node(package='nav2_collision_monitor', executable='collision_monitor', name='collision_monitor', parameters=[PARAMS], output='screen'),

        # ✅ No docking server!
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_navigation',
            output='screen',
            parameters=[{
                'use_sim_time': False,
                'autostart': True,
                'node_names': [
                    'controller_server',
                    'smoother_server',
                    'planner_server',
                    'behavior_server',
                    'bt_navigator',
                    'waypoint_follower',
                    'velocity_smoother',
                    'collision_monitor',
                ],
            }],
        ),
    ])
