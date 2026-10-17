"""
ros2 launch so101_bringup ros2_control.launch.py \
arm_type:=leader \
port:=/dev/ttyACM1
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    arm_type = LaunchConfiguration("arm_type")
    port = LaunchConfiguration("port")
    config_path = LaunchConfiguration("config_path")
    speed = LaunchConfiguration("speed")
    acc = LaunchConfiguration("acc")
    torque_limit = LaunchConfiguration("torque_limit")

    description_package = FindPackageShare("so_arm_description")

    xacro_file = PathJoinSubstitution(
        [description_package, "urdf", "so_arm.urdf.xacro"]
    )
    controllers_file = PathJoinSubstitution(
        [description_package, "config", "controllers.yaml"]
    )

    robot_description = {
        "robot_description": Command(
            [
                "xacro ",
                xacro_file,
                " arm_type:=",
                arm_type,
                " port:=",
                port,
                " config_path:=",
                config_path,
                " speed:=",
                speed,
                " acc:=",
                acc,
                " torque_limit:=",
                torque_limit,
            ]
        )
    }

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "arm_type",
                default_value="follower",
            ),
            DeclareLaunchArgument(
                "port",
                default_value="/dev/ttyUSB0",
            ),
            DeclareLaunchArgument(
                "config_path",
                default_value="",
            ),
            DeclareLaunchArgument(
                "speed",
                default_value="300",
            ),
            DeclareLaunchArgument(
                "acc",
                default_value="20",
            ),
            DeclareLaunchArgument(
                "torque_limit",
                default_value="300",
            ),
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                output="screen",
                parameters=[robot_description],
            ),
            Node(
                package="controller_manager",
                executable="ros2_control_node",
                output="screen",
                parameters=[
                    robot_description,
                    controllers_file,
                ],
            ),
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=[
                    "joint_state_broadcaster",
                    "--controller-manager",
                    "/controller_manager",
                ],
                output="screen",
            ),
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=[
                    "so101_arm_controller",
                    "--controller-manager",
                    "/controller_manager",
                ],
                output="screen",
            ),
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=[
                    "so101_gripper_controller",
                    "--controller-manager",
                    "/controller_manager",
                ],
                output="screen",
            ),
        ]
    )
