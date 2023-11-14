import launch
import launch_ros.actions

def generate_launch_description():
    return launch.LaunchDescription([
        launch.actions.DeclareLaunchArgument('output', default_value='screen'),
        launch_ros.actions.Node(
            package='yasmin_viewer',
            executable='yasmin_viewer_node',
            output='screen',
            name='yasmin_viewer_node'
        ),
        launch.actions.ExecuteProcess(
            cmd=['gnome-terminal', '-x', 'ros2', 'run', 'supervisor_node', 'supervisor_node'],
            output='screen',
            name='supervisor_node'
        ),
        launch.actions.ExecuteProcess(
            cmd=['gnome-terminal', '-x', 'ros2', 'run', 'supervisor_node', 'external_state_selector_node'],
            output='screen',
            name='external_state_selector_node'
        )
    ])
