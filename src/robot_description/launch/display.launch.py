import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_dir = get_package_share_directory('robot_description')
    default_model_path = os.path.join(pkg_dir, 'urdf', 'tiangong_lite.urdf')

    model_arg = DeclareLaunchArgument(
        'model',
        default_value=default_model_path,
        description='Path to robot URDF file'
    )

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[{
            'robot_description': Command(['xacro ', LaunchConfiguration('model')])
        }],
    )

    joint_state_publisher_gui_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
    )

    return LaunchDescription([
        model_arg,
        robot_state_publisher_node,
        joint_state_publisher_gui_node,
    ])
