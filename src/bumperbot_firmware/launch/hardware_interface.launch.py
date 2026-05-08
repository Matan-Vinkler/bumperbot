from launch import LaunchDescription
from launch.substitutions import Command

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

from ament_index_python import get_package_share_directory

import os

def generate_launch_description():
    bumperbot_description_dir = get_package_share_directory("bumperbot_description")
    bumperbot_controller_dir = get_package_share_directory("bumperbot_controller")

    robot_description = ParameterValue(Command([
        "xacro ", 
        os.path.join(bumperbot_description_dir, "urdf", "bumperbot.urdf.xacro"),
        " is_sim:=False"
    ]), 
    value_type=str)

    controller_manager = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            {"robot_description": robot_description,
            "use_sim_time": False},
            os.path.join(bumperbot_controller_dir, "config", "bumperbot_controllers.yaml")
        ]
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{"robot_description": robot_description}]
    )

    return LaunchDescription([
        robot_description,
        controller_manager,
        robot_state_publisher
    ])