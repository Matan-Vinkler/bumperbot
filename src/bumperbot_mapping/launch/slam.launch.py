from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    lifecycle_nodes = ["map_saver_server"]

    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time",
        default_value="true"
    )

    slam_config_arg = DeclareLaunchArgument(
        "slam_config",
        default_value=os.path.join(get_package_share_directory("bumperbot_mapping"), "config", "slam_toolbox.yaml")
    )

    localization_config_arg = DeclareLaunchArgument(
        "localization_config",
        default_value=os.path.join(get_package_share_directory("bumperbot_localization"), "config", "ekf_real.yaml")
    )

    use_sim_time = LaunchConfiguration("use_sim_time")
    slam_config = LaunchConfiguration("slam_config")
    localization_config = LaunchConfiguration("localization_config")

    imu_static_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        arguments=["--x", "0", "--y", "0", "--z", "0.103",
                   "--qx", "0", "--qy", "0", "--qz", "0", "--qw", "1",
                   "--frame-id", "base_footprint", "--child-frame-id", "imu_link_ekf"]
    )

    imu_republisher = Node(
        package="bumperbot_localization",
        executable="imu_republisher",
        output="screen"
    )

    robot_localization = Node(
        package="robot_localization",
        executable="ekf_node",
        name="ekf_filter_node",
        output="screen",
        parameters=[
            localization_config,
            {"use_sim_time": use_sim_time}
        ]
    )

    slam_toolbox = Node(
        package="slam_toolbox",
        executable="sync_slam_toolbox_node",
        name="slam_toolbox",
        output="screen",
        parameters=[
            slam_config,
            {"use_sim_time": use_sim_time}
        ]
    )

    nav2_map_saver = Node(
        package="nav2_map_server",
        executable="map_saver_server",
        name="map_saver_server",
        output="screen",
        parameters=[
            {"save_map_timeout": 5.0},
            {"use_sim_time": use_sim_time},
            {"free_thresh_default": 0.196},
            {"occupied_thresh_default": 0.65}
        ]
    )

    nav2_lifecycle_manager = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="lifecycle_manager",
        output="screen",
        parameters=[
            {"node_names": lifecycle_nodes},
            {"use_sim_time": use_sim_time},
            {"autostart": True},
        ]
    )

    # The EKF publishes the filtered odom→base_footprint TF. We must disable
    # the diff_drive_controller's competing TF publish so base_footprint has
    # only one parent in the TF tree (otherwise last-writer-wins at 100 Hz
    # vs 15 Hz causes slam_toolbox to lose the odom→base_footprint chain).
    disable_controller_tf = ExecuteProcess(
        cmd=[
            "bash", "-c",
            "until ros2 param get /bumperbot_controller enable_odom_tf > /dev/null 2>&1; "
            "do sleep 0.5; done && "
            "ros2 param set /bumperbot_controller enable_odom_tf false"
        ],
        output="screen"
    )

    launch_items = [
        use_sim_time_arg,
        slam_config_arg,
        localization_config_arg,
        imu_static_tf,
        imu_republisher,
        robot_localization,
        disable_controller_tf,
        slam_toolbox,
        nav2_map_saver,
        nav2_lifecycle_manager
    ]

    # On Jazzy, slam_toolbox is a lifecycle node that must be explicitly
    # configured and activated. The nav2_lifecycle_manager times out waiting
    # for the bond on RPi hardware, so we trigger the transitions manually.
    if os.environ.get("ROS_DISTRO") != "humble":
        launch_items.append(
            TimerAction(
                period=3.0,
                actions=[
                    ExecuteProcess(
                        cmd=[
                            "bash", "-c",
                            "until ros2 lifecycle get /slam_toolbox 2>/dev/null | grep -q .; "
                            "do sleep 0.5; done && "
                            "ros2 lifecycle set /slam_toolbox configure && "
                            "ros2 lifecycle set /slam_toolbox activate"
                        ],
                        output="screen"
                    )
                ]
            )
        )

    return LaunchDescription(launch_items)
