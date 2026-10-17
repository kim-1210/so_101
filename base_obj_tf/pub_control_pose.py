"""물체 자세와 로봇 그리퍼 목표 자세를 ROS 2로 발행한다."""

import time

import numpy as np
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float64MultiArray
from rclpy.node import Node
from scipy.spatial.transform import Rotation


OBJECT_TOPIC_NAME = "/obj_base"
GRIPPER_TARGET_TOPIC_NAME = "/gripper_target"
BASE_FRAME_ID = "base"
POSE_PRINT_INTERVAL_SECONDS = 1.0

# 측정한 y=-0.125 m를 실제 목표 y=-0.170 m에 맞추는 base 좌표 보정값
BASE_POSITION_CORRECTION = np.array([0.0, 0.0, 0.0])

# 물체의 PCA 회전은 흔들릴 수 있으므로 그리퍼 회전은 고정한다.
GRIPPER_TARGET_RPY = np.array([0.0, 0.0, 0.0])


class ObjectPosePublisher(Node):
    """4x4 자세 행렬을 두 종류의 PoseStamped 메시지로 발행한다."""

    def __init__(self) -> None:
        super().__init__("object_pose_publisher")
        self.object_publisher = self.create_publisher(
            PoseStamped,
            OBJECT_TOPIC_NAME,
            10,
        )
        self.gripper_target_publisher = self.create_publisher(
            PoseStamped,
            GRIPPER_TARGET_TOPIC_NAME,
            10,
        )
        self.last_pose_print_time = 0.0

        self.get_logger().info(
            f"Publishing object pose on {OBJECT_TOPIC_NAME} and "
            f"gripper_tcp target on {GRIPPER_TARGET_TOPIC_NAME}"
        )

    def publish_object_and_gripper_pose(
        self,
        base_from_object: np.ndarray,
    ) -> None:
        """물체 자세와 계산된 그리퍼 목표를 함께 발행한다."""
        object_position = base_from_object[:3, 3]
        object_rotation = Rotation.from_matrix(base_from_object[:3, :3])

        gripper_rotation = Rotation.from_euler("xyz", GRIPPER_TARGET_RPY)
        gripper_position = self._calculate_gripper_position(object_position)

        stamp = self.get_clock().now().to_msg()
        object_message = self._make_pose_message(
            stamp,
            object_position,
            object_rotation.as_quat(),
        )
        gripper_message = self._make_pose_message(
            stamp,
            gripper_position,
            gripper_rotation.as_quat(),
        )

        self.object_publisher.publish(object_message)
        self.gripper_target_publisher.publish(gripper_message)
        self._print_poses_at_interval(
            object_position,
            object_rotation,
            gripper_position,
        )

    @staticmethod
    def _calculate_gripper_position(
        object_position: np.ndarray,
    ) -> np.ndarray:
        """gripper_tcp가 도달할 base 기준 목표 위치를 계산한다."""
        return object_position + BASE_POSITION_CORRECTION

    @staticmethod
    def _make_pose_message(stamp, position, quaternion) -> PoseStamped:
        message = PoseStamped()
        message.header.stamp = stamp
        message.header.frame_id = BASE_FRAME_ID

        message.pose.position.x = float(position[0])
        message.pose.position.y = float(position[1])
        message.pose.position.z = float(position[2])
        message.pose.orientation.x = float(quaternion[0])
        message.pose.orientation.y = float(quaternion[1])
        message.pose.orientation.z = float(quaternion[2])
        message.pose.orientation.w = float(quaternion[3])
        return message

    def _print_poses_at_interval(
        self,
        object_position: np.ndarray,
        object_rotation: Rotation,
        gripper_position: np.ndarray,
    ) -> None:
        now = time.monotonic()
        if now - self.last_pose_print_time < POSE_PRINT_INTERVAL_SECONDS:
            return

        roll, pitch, yaw = object_rotation.as_euler("xyz", degrees=False)
        print(
            "Object pose | "
            f"x={object_position[0]:.3f}, "
            f"y={object_position[1]:.3f}, "
            f"z={object_position[2]:.3f} m | "
            f"roll={roll:.3f}, pitch={pitch:.3f}, yaw={yaw:.3f} rad\n"
            "Gripper TCP target | "
            f"x={gripper_position[0]:.3f}, "
            f"y={gripper_position[1]:.3f}, "
            f"z={gripper_position[2]:.3f} m | "
            "roll=0.000, pitch=0.000, yaw=0.000 rad",
            flush=True,
        )
        self.last_pose_print_time = now

    # 예전 호출 코드와의 호환을 위해 남긴 이름이다.
    def publish_matrix(self, transform: np.ndarray) -> None:
        self.publish_object_and_gripper_pose(transform)
