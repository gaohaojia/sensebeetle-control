import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('dr16_dbus'), 'config', 'dbus_serial_driver.yaml')

    dr16_dbus_node = Node(
        package='dr16_dbus',
        executable='dr16_dbus_node',
        name='dr16_dbus',
        namespace='sl_beetle',
        output='screen',
        emulate_tty=True,
        parameters=[config],
    )

    return LaunchDescription([dr16_dbus_node])
