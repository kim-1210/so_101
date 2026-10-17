"""물체 자세와 로봇 그리퍼 목표 자세를 ROS 2로 발행한다."""

import time

import numpy as np
from geometry_msgs.msg import Pose, PoseArray
from rclpy.node import Node
from scipy.spatial.transform import Rotation

BASKET_TOPIC_NAME = "/basket"
OBJECT_TOPIC_NAME = "/obj_bases"
GRIPPER_TARGET_TOPIC_NAME = "/gripper_targets"
BASE_FRAME_ID = "base"
POSE_PRINT_INTERVAL_SECONDS = 1.0

# 측정한 y=-0.125 m를 실제 목표 y=-0.170 m에 맞추는 base 좌표 보정값
BASE_POSITION_CORRECTION = np.array([0.0, 0.0, 0.0])

# 물체의 PCA 회전은 흔들릴 수 있으므로 그리퍼 회전은 고정한다.
GRIPPER_TARGET_RPY = np.array([0.0, 0.0, 0.0])


class ObjectPosePublishers(Node):
    """여러 4x4 자세 행렬을 두 종류의 PoseArray 메시지로 발행한다."""

    def __init__(self) -> None:
        super().__init__("object_pose_publisher")
        self.baseket_publisher = self.create_publisher(
            Pose,
            BASKET_TOPIC_NAME,
            10,
        )
        self.object_publisher = self.create_publisher(
            PoseArray,
            OBJECT_TOPIC_NAME,
            10,
        )
        self.gripper_target_publisher = self.create_publisher(
            PoseArray,
            GRIPPER_TARGET_TOPIC_NAME,
            10,
        )
        self.last_pose_print_time = 0.0

        self.get_logger().info(
            f"Publishing object pose on {OBJECT_TOPIC_NAME} and "
            f"gripper_tcp target on {GRIPPER_TARGET_TOPIC_NAME}"
        )

    def publisher_basket(
        self,
        basket_position, # (x, y, z)
        basket_rotation, # (rx, ry, rz, rw) 
    ):
        msg = Pose()
        msg.position.x, msg.position.y, msg.position.z = basket_position
        msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w = basket_rotation
        self.baseket_publisher.publish(
            msg
        )

    def publish_object_and_gripper_poses(
        self,
        base_from_objects,
    ) -> None:
        """여러 물체 자세와 계산된 그리퍼 목표들을 함께 발행한다."""
        object_message = self._make_pose_array_message()
        gripper_message = self._make_pose_array_message()
        object_infos = []

        for base_from_object in base_from_objects:
            object_position = base_from_object[:3, 3]
            object_rotation = Rotation.from_matrix(base_from_object[:3, :3])

            gripper_rotation = Rotation.from_euler("xyz", GRIPPER_TARGET_RPY)
            gripper_position = self._calculate_gripper_position(object_position)

            object_message.poses.append(
                self._make_pose(object_position, object_rotation.as_quat())
            )
            gripper_message.poses.append(
                self._make_pose(gripper_position, gripper_rotation.as_quat())
            )
            object_infos.append(
                (object_position, object_rotation, gripper_position)
            )

        self.object_publisher.publish(object_message)
        self.gripper_target_publisher.publish(gripper_message)
        self._print_poses_at_interval(object_infos)

    def publish_object_and_gripper_pose(
        self,
        base_from_object: np.ndarray,
    ) -> None:
        """물체 1개도 PoseArray에 1개짜리 리스트로 담아 발행한다."""
        self.publish_object_and_gripper_poses([base_from_object])

    @staticmethod
    def _calculate_gripper_position(
        object_position: np.ndarray,
    ) -> np.ndarray:
        """gripper_tcp가 도달할 base 기준 목표 위치를 계산한다."""
        return object_position + BASE_POSITION_CORRECTION

    def _make_pose_array_message(self) -> PoseArray:
        message = PoseArray()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = BASE_FRAME_ID
        return message

    @staticmethod
    def _make_pose(position, quaternion) -> Pose:
        message = Pose()
        message.position.x = float(position[0])
        message.position.y = float(position[1])
        message.position.z = float(position[2])
        message.orientation.x = float(quaternion[0])
        message.orientation.y = float(quaternion[1])
        message.orientation.z = float(quaternion[2])
        message.orientation.w = float(quaternion[3])
        return message

    def _print_poses_at_interval(
        self,
        object_infos,
    ) -> None:
        now = time.monotonic()
        if now - self.last_pose_print_time < POSE_PRINT_INTERVAL_SECONDS:
            return

        if not object_infos:
            print("Object poses | count=0", flush=True)
            self.last_pose_print_time = now
            return

        lines = [f"Object poses | count={len(object_infos)}"]
        for index, (
            object_position,
            object_rotation,
            gripper_position,
        ) in enumerate(object_infos):
            roll, pitch, yaw = object_rotation.as_euler("xyz", degrees=False)
            lines.append(
                f"[{index}] Object | "
                f"x={object_position[0]:.3f}, "
                f"y={object_position[1]:.3f}, "
                f"z={object_position[2]:.3f} m | "
                f"roll={roll:.3f}, pitch={pitch:.3f}, yaw={yaw:.3f} rad"
            )
            lines.append(
                f"[{index}] Gripper TCP target | "
                f"x={gripper_position[0]:.3f}, "
                f"y={gripper_position[1]:.3f}, "
                f"z={gripper_position[2]:.3f} m | "
                "roll=0.000, pitch=0.000, yaw=0.000 rad"
            )

        print(
            "\n".join(lines),
            flush=True,
        )
        self.last_pose_print_time = now

    # 예전 호출 코드와의 호환을 위해 남긴 이름이다.
    def publish_matrix(self, transform: np.ndarray) -> None:
        self.publish_object_and_gripper_pose(transform)
