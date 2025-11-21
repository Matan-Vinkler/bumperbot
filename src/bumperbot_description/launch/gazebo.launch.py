from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable, IncludeLaunchDescription
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

from ament_index_python import get_package_share_directory

import os
from pathlib import Path

def generate_launch_description():
    bumperbot_description_dir = get_package_share_directory("bumperbot_description")

    model_arg = DeclareLaunchArgument(
        name="model",
        default_value=os.path.join(bumperbot_description_dir, "urdf", "bumperbot.urdf.xacro"),
        description="Absolute path to robot urdf file"
    )

    world_name_arg = DeclareLaunchArgument(
        name="world_name",
        default_value="empty"
    )

    world_path = PathJoinSubstitution([
        bumperbot_description_dir,
        "worlds",
        PythonExpression(expression=["'", LaunchConfiguration("world_name"), "'", " +  '.world'"])
    ])

    robot_description = ParameterValue(Command([
        "xacro ", 
        LaunchConfiguration("model")
    ]), 
    value_type=str)

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{"robot_description": robot_description}]
    )

    model_path = str(Path(bumperbot_description_dir).parent.resolve())
    model_path += os.pathsep + os.path.join(bumperbot_description_dir, "models")

    gazebo_resource_path = SetEnvironmentVariable(
        "GZ_SIM_RESOURCE_PATH", model_path
    )

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(get_package_share_directory("ros_gz_sim"), "launch"), "/gz_sim.launch.py"]),
        launch_arguments={
            "gz_args": PythonExpression(expression=["'", world_path ," -v 4 -r'"])
        }.items()
    )

    gz_spawn_entity = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=["-topic", "robot_description", "-name", "bumperbot"],
        output="screen"
    )

    gz_ros2_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",
            "/imu@sensor_msgs/msg/Imu[gz.msgs.IMU",
            "/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan"
        ],
        remappings=[
            ("/imu", "/imu/out")
        ]
    )

    return LaunchDescription([
        model_arg,
        world_name_arg,
        robot_state_publisher,
        gazebo_resource_path,
        gazebo,
        gz_spawn_entity,
        gz_ros2_bridge
    ])