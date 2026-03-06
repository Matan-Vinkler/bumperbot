import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node

from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time",
        default_value="True"
    )
    use_sim_time = LaunchConfiguration("use_sim_time")

    project_dir = get_package_share_directory("bumperbot_navigation")
    lifecycle_nodes = ["controller_server", "planner_server", "smoother_server", "bt_navigator"]

    nav2_controller_server = Node(
        package="nav2_controller",
        executable="controller_server",
        name="controller_server",
        output="screen",
        parameters=[
            os.path.join(project_dir, "config", "controller_server.yaml"),
            {"use_sim_time": use_sim_time}
        ]
    )

    nav2_planner_server = Node(
        package="nav2_planner",
        executable="planner_server",
        name="planner_server",
        output="screen",
        parameters=[
            os.path.join(project_dir, "config", "planner_server.yaml"),
            {"use_sim_time": use_sim_time}
        ]
    )

    nav2_smoother_server = Node(
        package="nav2_smoother",
        executable="smoother_server",
        name="smoother_server",
        output="screen",
        parameters=[
            os.path.join(project_dir, "config", "smoother_server.yaml"),
            {"use_sim_time": use_sim_time}
        ]
    )

    bt_xml_path = os.path.join(project_dir, "behavior_tree", "bt_navigation.xml")
    nav2_bt_navigator = Node(
        package="nav2_bt_navigator",
        executable="bt_navigator",
        name="bt_navigator",
        output="screen",
        parameters=[
            os.path.join(project_dir, "config", "bt_navigator.yaml"),
            {"use_sim_time": use_sim_time},
            {"default_nav_to_pose_bt_xml": bt_xml_path},
            {"default_nav_through_poses_bt_xml": bt_xml_path}
        ]
    )

    nav2_lifecycle_manager = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="lifecycle_manager_navigation",
        output="screen",
        parameters=[
            {"node_names": lifecycle_nodes},
            {"use_sim_time": use_sim_time},
            {"autostart": True}
        ]
    )

    return LaunchDescription([
        use_sim_time_arg,
        nav2_controller_server,
        nav2_planner_server,
        nav2_smoother_server,
        nav2_bt_navigator,
        nav2_lifecycle_manager
    ])