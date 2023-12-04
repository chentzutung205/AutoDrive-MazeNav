from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='bb8_final',
            executable='camera_testing',
            name='camera_testing_node'
        ),
        Node(
            package='bb8_final',
            executable='trained_knn',
            name='trained_knn_node'
        ),
        Node(
            package='bb8_final',
            executable='surrounding',
            name='get_wall_node'
        ),
        Node(
            package='bb8_final',
            executable='frontal_viewer',
            name='get_object_node'
        ),
        Node(
            package='bb8_final',
            executable='controller',
            name='controller_node'
        )
        # Add more nodes as needed
    ])
