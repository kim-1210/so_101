"""
ros2 launch so101_bringup moveit_follower.launch.py \
follower_port:=/dev/ttyUSB0

ros2 run so101_moveit_config moveit_pose_goal --ros-args \
-p frame_id:=base \
-p x:=0.15 \
-p y:=-0.3 \
-p z:=0.18 \
-p execute:=true

ros2 run so101_moveit_config moveit_pose_goal --ros-args \
  -p frame_id:=base \
  -p x:=0.21 \
  -p y:=-0.191 \
  -p z:=0.15 \
  -p roll:=0.0 \
  -p pitch:=0.0 \
  -p yaw:=0.0 \
  -p position_only:=true \
  -p execute:=true
"""

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    IncludeLaunchDescription,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node, SetRemap
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    follower_port = LaunchConfiguration('follower_port')
    follower_config_path = LaunchConfiguration('follower_config_path')
    rate_hz = LaunchConfiguration('rate_hz')
    speed = LaunchConfiguration('speed')
    acc = LaunchConfiguration('acc')
    torque_limit = LaunchConfiguration('torque_limit')

    default_config = PathJoinSubstitution([
        FindPackageShare('so101_hardware'),
        'config',
        'so101_follower.json',
    ])

    hardware = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([
            FindPackageShare('so101_bringup'),
            'launch',
            'only_hardware_follower.launch.py',
        ])),
        launch_arguments={
            'follower_port': follower_port,
            'follower_config_path': follower_config_path,
            'rate_hz': rate_hz,
            'speed': speed,
            'acc': acc,
            'torque_limit': torque_limit,
        }.items(),
    )

    # move_group listens to /joint_states by default, while the hardware node
    # publishes /follower/joint_states.  Scope the remap to MoveIt so it always
    # plans from the real arm state.
    move_group = GroupAction([
        SetRemap(src='/joint_states', dst='/follower/joint_states'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution([
                FindPackageShare('so101_moveit_config'),
                'launch',
                'move_group.launch.py',
            ])),
        ),
    ])

    bridge = Node(
        package='so101_bringup',
        executable='follow_joint_trajectory_bridge',
        name='trajectory_bridge',
        output='screen',
    )

    return LaunchDescription([
        DeclareLaunchArgument('follower_port', default_value='/dev/ttyUSB0'),
        DeclareLaunchArgument(
            'follower_config_path', default_value=default_config),
        DeclareLaunchArgument('rate_hz', default_value='20.0'),
        DeclareLaunchArgument('speed', default_value='300'),
        DeclareLaunchArgument('acc', default_value='20'),
        DeclareLaunchArgument('torque_limit', default_value='300'),
        hardware,
        move_group,
        bridge,
    ])
