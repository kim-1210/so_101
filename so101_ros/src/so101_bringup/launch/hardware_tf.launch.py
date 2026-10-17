"""
ros2 launch so101_bringup hardware_tf.launch.py \
leader_port:=/dev/ttyACM1 \
follower_port:=/dev/ttyACM0 \
leader_config_path:=/media/kim/hdd1tb2/master_study/so_101/so101_ros/src/so101_hardware/config/so101_leader.json \
follower_config_path:=/media/kim/hdd1tb2/master_study/so_101/so101_ros/src/so101_hardware/config/so101_follower.json \
speed:=1200 acc:=50
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    # =========================
    # Launch Arguments
    # =========================
    leader_port = LaunchConfiguration("leader_port")
    follower_port = LaunchConfiguration("follower_port")
    leader_config_path = LaunchConfiguration("leader_config_path")
    follower_config_path = LaunchConfiguration("follower_config_path")
    rate_hz = LaunchConfiguration("rate_hz")
    speed = LaunchConfiguration("speed")
    acc = LaunchConfiguration("acc")
    torque_limit = LaunchConfiguration("torque_limit")

    # URDF/Xacro 경로
    urdf_path = LaunchConfiguration("urdf_path")

    # =========================
    # robot_description 생성
    # =========================
    # xacro 파일을 읽어서 robot_description 문자열로 변환
    robot_description = ParameterValue(
        Command(["xacro ", urdf_path]),
        value_type=str,
    )

    return LaunchDescription([
        # =========================
        # 기존 hardware launch arguments
        # =========================
        DeclareLaunchArgument(
            "leader_port",
            default_value="/dev/ttyACM0",
        ),
        DeclareLaunchArgument(
            "follower_port",
            default_value="/dev/ttyUSB0",
        ),
        DeclareLaunchArgument(
            "leader_config_path",
            default_value="/home/sclab/Desktop/so101_ros/src/so101_hardware/config/so101_leader.json",
        ),
        DeclareLaunchArgument(
            "follower_config_path",
            default_value="/home/sclab/Desktop/so101_ros/src/so101_hardware/config/so101_follower.json",
        ),
        DeclareLaunchArgument(
            "rate_hz",
            default_value="20.0",
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

        # =========================
        # URDF/Xacro 경로 argument
        # =========================
        DeclareLaunchArgument(
            "urdf_path",
            default_value=PathJoinSubstitution([
                FindPackageShare("so_arm_description"),
                "urdf",
                "so_arm.urdf.xacro",
            ]),
            description="SO101 robot URDF/Xacro path",
        ),

        # =========================
        # Leader hardware node
        # =========================
        Node(
            package="so101_hardware",
            executable="hardware_node",
            namespace="leader",
            name="hardware",
            output="screen",
            parameters=[{
                "arm_type": "leader",
                "port": leader_port,
                "config_path": leader_config_path,
                "rate_hz": rate_hz,
                "speed": speed,
                "acc": acc,
                "torque_limit": torque_limit,
            }],
        ),

        # =========================
        # Follower hardware node
        # =========================
        Node(
            package="so101_hardware",
            executable="hardware_node",
            namespace="follower",
            name="hardware",
            output="screen",
            parameters=[{
                "arm_type": "follower",
                "port": follower_port,
                "config_path": follower_config_path,
                "rate_hz": rate_hz,
                "speed": speed,
                "acc": acc,
                "torque_limit": torque_limit,
            }],
        ),

        # =========================
        # /tf, /tf_static 생성 노드
        # =========================
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            name="robot_state_publisher",
            output="screen",
            parameters=[{
                "robot_description": robot_description,
            }],
            remappings=[
                # robot_state_publisher는 기본적으로 joint_states를 봄
                # 네 hardware node는 /follower/joint_states를 publish하므로 remap 필요
                ("joint_states", "/follower/joint_states"),
                ("tf", "/tf"),
                ("tf_static", "/tf_static"),
            ],
        ),
    ])
