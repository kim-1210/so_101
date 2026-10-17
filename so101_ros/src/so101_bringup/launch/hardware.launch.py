"""
ros2 launch so101_bringup hardware.launch.py \
leader_port:=/dev/ttyACM2 \
follower_port:=/dev/ttyACM1 \
leader_config_path:=/home/sclab/Desktop/so101_ros/src/so101_hardware/config/so101_leader.json \
follower_config_path:=/home/sclab/Desktop/so101_ros/src/so101_hardware/config/so101_follower.json
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# 해당 함수는 먼저 있다는 가정하에 LaunchConfiguration를 설정 후 
# DeclareLaunchArgument로 정의함
def generate_launch_description():
    # "leader_port라는 arg를 참조하는 핸들(handle)" -> 실제값을 읽는게 아님
    leader_port = LaunchConfiguration("leader_port")
    follower_port = LaunchConfiguration("follower_port")
    leader_config_path = LaunchConfiguration("leader_config_path")
    follower_config_path = LaunchConfiguration("follower_config_path")
    rate_hz = LaunchConfiguration("rate_hz")
    speed = LaunchConfiguration("speed")
    acc = LaunchConfiguration("acc")
    torque_limit = LaunchConfiguration("torque_limit")

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
    ])
