from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    with open('/home/sang/ros2_ws/src/robot_arm_description/urdf/test_box.urdf', 'r') as urdf_file:
        robot_description = urdf_file.read()

    return LaunchDescription([
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_description}]
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen'
        )
    ])

