"""
TF를 보면서 캘리브레이션할 때 쓰는 launch입니다.

hardware_tf.launch.py와 거의 같지만, hardware_node에 enable_torque:=false를 넘겨서
모터 토크를 끕니다. 그래서 손으로 로봇을 움직이기 쉽습니다.

예시:
ros2 launch so101_bringup hardware_tf_no_torque.launch.py \
leader_port:=/dev/ttyACM1 \
follower_port:=/dev/ttyACM0 \
leader_config_path:=/media/kim/hdd1tb2/master_study/so_101/so101_ros/src/so101_hardware/config/so101_leader.json \
follower_config_path:=/media/kim/hdd1tb2/master_study/so_101/so101_ros/src/so101_hardware/config/so101_follower.json

ros2 run robot_state_publisher robot_state_publisher   --ros-args   -p robot_description:="$(xacro /home/sclab/Desktop/so101_ros/src/so_arm_description/urdf/so_arm.urdf.xacro)"   -r joint_states:=/follower/joint_states
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    leader_port = LaunchConfiguration("leader_port")
    follower_port = LaunchConfiguration("follower_port")
    leader_config_path = LaunchConfiguration("leader_config_path")
    follower_config_path = LaunchConfiguration("follower_config_path")
    rate_hz = LaunchConfiguration("rate_hz")
    urdf_path = LaunchConfiguration("urdf_path")

    robot_description = ParameterValue(
        Command(["xacro ", urdf_path]),
        value_type=str,
    )

    return LaunchDescription([
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
            "urdf_path",
            default_value=PathJoinSubstitution([
                FindPackageShare("so_arm_description"),
                "urdf",
                "so_arm.urdf.xacro",
            ]),
            description="SO101 robot URDF/Xacro path",
        ),
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
                "enable_torque": False,
            }],
        ),
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
                "enable_torque": False,
            }],
        ),
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            name="robot_state_publisher",
            output="screen",
            parameters=[{
                "robot_description": robot_description,
            }],
            remappings=[
                ("joint_states", "/follower/joint_states"),
                ("tf", "/tf"),
                ("tf_static", "/tf_static"),
            ],
        ),
    ])
