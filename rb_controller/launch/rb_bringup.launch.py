import os
from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node


def generate_launch_description():

    urdf_file_name = "rb5.urdf"
    urdf_path = os.path.join(get_package_share_directory("rb_description"), "urdf", urdf_file_name)
    with open(urdf_path, "r") as infp:
        robot_desc = infp.read()

    rviz_config_file_name = "rviz_config.rviz"
    rviz_config_path = os.path.join(get_package_share_directory("rb_description"), "rviz", rviz_config_file_name)

    robot_controller_node = Node(
        package="rb_controller",
        executable="controller",
        output="screen",
    )

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[{"robot_description": robot_desc}],
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_path],
    )

    node_to_start = [robot_controller_node, robot_state_publisher_node, rviz_node]

    return LaunchDescription(node_to_start)
