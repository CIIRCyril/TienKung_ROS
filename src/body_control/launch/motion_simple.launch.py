from launch import LaunchDescription
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    body_control_dir = get_package_share_directory('body_control')

    container = ComposableNodeContainer(
        name='body_control_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[
            ComposableNode(
                package='body_control',
                plugin='body_control::BodyControl',
                name='BodyControl',
                parameters=[{
                    'config_file': os.path.join(
                        body_control_dir, 'param', 'config.yaml'),
                    'motor_setting_file': os.path.join(
                        body_control_dir, 'param', 'motor_setting_simple.yaml'),
                    'imu_setting_file': os.path.join(
                        body_control_dir, 'param', 'xsens_imu_setting.yaml'),
                    'power_setting_file': os.path.join(
                        body_control_dir, 'param', 'power_setting_evt.yaml'),
                }],
            ),
            ComposableNode(
                package='body_control',
                plugin='body_control::MonitorPlugin',
                name='Monitor',
                parameters=[{
                    'motor_setting_file': os.path.join(
                        body_control_dir, 'param', 'motor_setting_simple.yaml'),
                }],
            ),
        ],
        output='screen',
    )

    usb_sbus_node = Node(
        package='usb_sbus',
        executable='usb_sbus_node',
        name='usb_sbus_node',
        output='screen',
    )

    return LaunchDescription([container, usb_sbus_node])
