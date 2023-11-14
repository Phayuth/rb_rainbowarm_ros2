import os
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    urdf = '/home/yuth/ws_yuthdev/ws_rbarm/src/rb_rainbowarm_ros2/rb_description/urdf/simple_robot.urdf'
    with open(urdf, 'r') as infp:
        robot_desc = infp.read()

    joint_state_publisher_node = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
    )
    robot_state_publisher_node = Node(
        package="robot_state_publisher", 
        executable="robot_state_publisher", 
        output="screen", 
        parameters=[{'robot_description': robot_desc}])

    return LaunchDescription([joint_state_publisher_node, robot_state_publisher_node])