from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    execute = LaunchConfiguration("execute")
    hold_seconds = LaunchConfiguration("hold_seconds")
    target_type = LaunchConfiguration("target_type")
    goal_x = LaunchConfiguration("goal_x")
    goal_y = LaunchConfiguration("goal_y")
    goal_z = LaunchConfiguration("goal_z")
    joint_rotation = LaunchConfiguration("joint_rotation")
    joint_pitch = LaunchConfiguration("joint_pitch")
    joint_elbow = LaunchConfiguration("joint_elbow")
    joint_wrist_pitch = LaunchConfiguration("joint_wrist_pitch")
    joint_wrist_roll = LaunchConfiguration("joint_wrist_roll")
    obstacle_x = LaunchConfiguration("obstacle_x")
    obstacle_y = LaunchConfiguration("obstacle_y")
    obstacle_z = LaunchConfiguration("obstacle_z")
    obstacle_size_x = LaunchConfiguration("obstacle_size_x")
    obstacle_size_y = LaunchConfiguration("obstacle_size_y")
    obstacle_size_z = LaunchConfiguration("obstacle_size_z")
    moveit_config = (
        MoveItConfigsBuilder("so_arm", package_name="so101_moveit_config")
        .to_moveit_configs()
    )

    node = Node(
        package="so101_moveit_test",
        executable="planning_scene_obstacle_demo",
        name="planning_scene_obstacle_demo",
        output="screen",
        parameters=[
            moveit_config.to_dict(),
            {
                "execute": ParameterValue(execute, value_type=bool),
                "publish_display_trajectory": True,
                "hold_seconds": ParameterValue(hold_seconds, value_type=float),
                "group_name": "arm",
                "end_effector_link": "gripper_tcp",
                "planning_frame": "base",
                "target_type": target_type,
                "goal_x": ParameterValue(goal_x, value_type=float),
                "goal_y": ParameterValue(goal_y, value_type=float),
                "goal_z": ParameterValue(goal_z, value_type=float),
                "joint_rotation": ParameterValue(joint_rotation, value_type=float),
                "joint_pitch": ParameterValue(joint_pitch, value_type=float),
                "joint_elbow": ParameterValue(joint_elbow, value_type=float),
                "joint_wrist_pitch": ParameterValue(joint_wrist_pitch, value_type=float),
                "joint_wrist_roll": ParameterValue(joint_wrist_roll, value_type=float),
                "obstacle_x": ParameterValue(obstacle_x, value_type=float),
                "obstacle_y": ParameterValue(obstacle_y, value_type=float),
                "obstacle_z": ParameterValue(obstacle_z, value_type=float),
                "obstacle_size_x": ParameterValue(obstacle_size_x, value_type=float),
                "obstacle_size_y": ParameterValue(obstacle_size_y, value_type=float),
                "obstacle_size_z": ParameterValue(obstacle_size_z, value_type=float),
                "planning_time": 10.0,
                "planning_attempts": 10,
                "velocity_scale": 0.15,
                "acceleration_scale": 0.15,
            },
        ],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "execute",
            default_value="false",
            description="Execute the planned path. Keep false when testing with real hardware first.",
        ),
        DeclareLaunchArgument("hold_seconds", default_value="8.0"),
        DeclareLaunchArgument("target_type", default_value="joint"),
        DeclareLaunchArgument("goal_x", default_value="0.15"),
        DeclareLaunchArgument("goal_y", default_value="-0.30"),
        DeclareLaunchArgument("goal_z", default_value="0.18"),
        DeclareLaunchArgument("joint_rotation", default_value="-0.90"),
        DeclareLaunchArgument("joint_pitch", default_value="0.75"),
        DeclareLaunchArgument("joint_elbow", default_value="-0.95"),
        DeclareLaunchArgument("joint_wrist_pitch", default_value="0.55"),
        DeclareLaunchArgument("joint_wrist_roll", default_value="0.0"),
        DeclareLaunchArgument("obstacle_x", default_value="0.20"),
        DeclareLaunchArgument("obstacle_y", default_value="-0.18"),
        DeclareLaunchArgument("obstacle_z", default_value="0.10"),
        DeclareLaunchArgument("obstacle_size_x", default_value="0.08"),
        DeclareLaunchArgument("obstacle_size_y", default_value="0.14"),
        DeclareLaunchArgument("obstacle_size_z", default_value="0.16"),
        node,
    ])
