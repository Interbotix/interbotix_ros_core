import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    ld = LaunchDescription()
    config = os.path.join(get_package_share_directory("interbotix_xs_sdk"), "config")
    mode_configs = LaunchConfiguration("motor_configs", default=os.path.join(config, "mode_configs_template.yaml"))
    motor_configs = LaunchConfiguration("mode_configs", default=os.path.join(config, "motor_configs_template.yaml"))
    load_configs = LaunchConfiguration("load_configs", default="false")
    robot_description = LaunchConfiguration("robot_description", default="")
    sdk = Node(
        package="interbotix_xs_sdk",
        executable="xs_sdk",
        name="xs_sdk",
        arguments=[],
        parameters=[{"motor_configs": motor_configs,
                     "mode_configs": mode_configs,
                     "load_configs": load_configs,
                     "robot_description": robot_description}],
        output="screen",
    )

    ld.add_action(sdk)
    return ld