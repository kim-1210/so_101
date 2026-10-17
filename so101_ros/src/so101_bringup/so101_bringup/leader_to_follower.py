"""
ros2 run so101_bringup leader_to_follower --ros-args \
-p publish_rate_hz:=20.0 \
-p max_step_rad:=3.14 \
-p command_offsets_rad:="[0.0, 0.0, 0.0, 0.0, 1.57, 0.0]"

publish_rate_hz -> 1초에 50번 명령 보냄
max_step_rad -> follower가 한 번 명령을 받을 때, 관절을 최대 몇 radian 만큼만 움직이게 할지 정하는 제한값
command_offsets_rad -> follower로 보내기 직전에 더할 관절별 보정값
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray


class LeaderToFollower(Node):
    def __init__(self):
        super().__init__("leader_to_follower")

        self.declare_parameter("leader_topic", "/leader/joint_states")
        self.declare_parameter("follower_topic", "/follower/joint_commands")
        self.declare_parameter("publish_rate_hz", 20.0)
        self.declare_parameter("max_step_rad", 0.08)
        self.declare_parameter("copy_gripper", True)
        self.declare_parameter("command_offsets_rad", [0.0, 0.0, 0.0, 0.0, 0.0, 0.0])

        self.leader_topic = self.get_parameter("leader_topic").value
        self.follower_topic = self.get_parameter("follower_topic").value
        self.publish_rate_hz = float(self.get_parameter("publish_rate_hz").value)
        self.max_step_rad = float(self.get_parameter("max_step_rad").value)
        self.copy_gripper = bool(self.get_parameter("copy_gripper").value)
        self.command_offsets_rad = list(self.get_parameter("command_offsets_rad").value)

        if len(self.command_offsets_rad) != 6:
            raise ValueError("command_offsets_rad must have exactly 6 values")

        self.latest_position = None
        self.last_command = None

        self.publisher = self.create_publisher(
            Float64MultiArray,
            self.follower_topic,
            1,
        )
        self.subscription = self.create_subscription(
            JointState,
            self.leader_topic,
            self.leader_callback,
            10,
        )

        period = 1.0 / self.publish_rate_hz
        self.timer = self.create_timer(period, self.publish_command)

        self.get_logger().info(
            f"Leader to follower relay started: {self.leader_topic} -> {self.follower_topic}"
        )

    def leader_callback(self, msg):
        if len(msg.position) < 6:
            self.get_logger().warn(
                f"Expected 6 leader joints, got {len(msg.position)}",
                throttle_duration_sec=2.0,
            )
            return

        self.latest_position = list(msg.position[:6])

    def publish_command(self):
        if self.latest_position is None:
            return

        if self.last_command is None:
            command = list(self.latest_position)
        else:
            command = [
                self.limit_step(previous, target)
                for previous, target in zip(self.last_command, self.latest_position)
            ]

        if not self.copy_gripper and self.last_command is not None:
            command[5] = self.last_command[5]

        command_with_offsets = [
            position + offset
            for position, offset in zip(command, self.command_offsets_rad)
        ]

        msg = Float64MultiArray()
        msg.data = command_with_offsets
        self.publisher.publish(msg)
        self.last_command = command

    def limit_step(self, previous, target):
        diff = target - previous

        if diff > self.max_step_rad:
            return previous + self.max_step_rad

        if diff < -self.max_step_rad:
            return previous - self.max_step_rad

        return target


def main():
    rclpy.init()
    node = LeaderToFollower()

    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
