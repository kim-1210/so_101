from launch_ros.actions import Node
from launch import LaunchDescription
from launch.substitutions import Command
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory
import os

import xacro

description_pkg = "so_arm_description"


def generate_launch_description():
    xacro_file = os.path.join(
        get_package_share_directory(description_pkg),
        "urdf",
        "so_arm.urdf.xacro"
    )
    rviz_config_file = os.path.join(
        get_package_share_directory(description_pkg),
        "rviz",
        "so_arm.rviz"
    )
    robot_description_config = xacro.process_file(xacro_file) # xacro -> urdf 변환

    robot_description = {
        "robot_description":
        robot_description_config.toxml()
    }

    # TF생성
    rsp = Node( 
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[robot_description]
    )

    joint_state_publisher_gui_node = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
        output="screen"
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["-d", rviz_config_file],
        output="screen"
    )

    return LaunchDescription([
        rsp,
        joint_state_publisher_gui_node,
        rviz_node
    ])
