"""
ros2 run so101_bringup leader_to_follower --ros-args \
-p publish_rate_hz:=50.0 \
-p max_step_rad:=0.8
"""

import time
from threading import Lock

from control_msgs.action import FollowJointTrajectory
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray


class FollowJointTrajectoryBridge(Node):

    HARDWARE_JOINTS = [
        'Rotation',
        'Pitch',
        'Elbow',
        'Wrist_Pitch',
        'Wrist_Roll',
        'Jaw',
    ]

    def __init__(self):
        super().__init__('so101_follow_joint_trajectory_bridge')

        self.declare_parameter('action_name',
                               '/so101_arm_controller/follow_joint_trajectory')
        self.declare_parameter('command_topic', '/follower/joint_commands')
        self.declare_parameter('joint_state_topic', '/follower/joint_states')
        self.declare_parameter('goal_tolerance', 0.08)
        self.declare_parameter('goal_time_tolerance', 3.0)

        action_name = self.get_parameter('action_name').value
        command_topic = self.get_parameter('command_topic').value
        state_topic = self.get_parameter('joint_state_topic').value

        self._positions = {}
        self._state_lock = Lock()
        self._active_goal = None

        self._command_pub = self.create_publisher(
            Float64MultiArray, command_topic, 10)
        self._state_sub = self.create_subscription(
            JointState, state_topic, self._state_callback, 10)
        self._action_server = ActionServer(
            self,
            FollowJointTrajectory,
            action_name,
            goal_callback=self._goal_callback,
            cancel_callback=self._cancel_callback,
            execute_callback=self._execute_callback,
            callback_group=ReentrantCallbackGroup(),
        )

        self.get_logger().info(
            f'Ready: {action_name} -> {command_topic}')

    def _state_callback(self, msg):
        with self._state_lock:
            self._positions.update(zip(msg.name, msg.position))

    def _goal_callback(self, goal_request):
        trajectory = goal_request.trajectory
        names = trajectory.joint_names

        if not trajectory.points:
            self.get_logger().error('Rejected an empty trajectory')
            return GoalResponse.REJECT
        if len(names) != len(set(names)):
            self.get_logger().error('Rejected duplicate joint names')
            return GoalResponse.REJECT
        if any(name not in self.HARDWARE_JOINTS for name in names):
            self.get_logger().error(
                f'Rejected unknown joints: {names}')
            return GoalResponse.REJECT
        if any(len(point.positions) != len(names)
               for point in trajectory.points):
            self.get_logger().error('Rejected invalid trajectory point size')
            return GoalResponse.REJECT
        if self._active_goal is not None:
            self.get_logger().warning('Rejected: another trajectory is active')
            return GoalResponse.REJECT

        return GoalResponse.ACCEPT

    def _cancel_callback(self, _goal_handle):
        return CancelResponse.ACCEPT

    def _hardware_command(self, names, positions):
        with self._state_lock:
            command = dict(self._positions)
        command.update(zip(names, positions))

        missing = [name for name in self.HARDWARE_JOINTS
                   if name not in command]
        if missing:
            return False, missing

        msg = Float64MultiArray()
        msg.data = [float(command[name]) for name in self.HARDWARE_JOINTS]
        self._command_pub.publish(msg)
        return True, []

    @staticmethod
    def _seconds(duration):
        return duration.sec + duration.nanosec * 1e-9

    def _execute_callback(self, goal_handle):
        self._active_goal = goal_handle
        result = FollowJointTrajectory.Result()
        trajectory = goal_handle.request.trajectory
        names = trajectory.joint_names
        previous_time = 0.0

        try:
            # Wait briefly for the first hardware state. It supplies Jaw when
            # MoveIt is planning only the five arm joints.
            for _ in range(20):
                with self._state_lock:
                    ready = all(name in self._positions
                                for name in self.HARDWARE_JOINTS)
                if ready:
                    break
                time.sleep(0.05)
            else:
                result.error_code = result.INVALID_GOAL
                result.error_string = 'No complete follower joint state received'
                goal_handle.abort()
                return result

            for point in trajectory.points:
                if goal_handle.is_cancel_requested:
                    goal_handle.canceled()
                    result.error_code = result.SUCCESSFUL
                    result.error_string = 'Trajectory canceled'
                    return result

                point_time = self._seconds(point.time_from_start)
                time.sleep(max(0.0, point_time - previous_time))
                previous_time = point_time

                published, missing = self._hardware_command(
                    names, point.positions)
                if not published:
                    result.error_code = result.INVALID_GOAL
                    result.error_string = f'Missing joint states: {missing}'
                    goal_handle.abort()
                    return result

                feedback = FollowJointTrajectory.Feedback()
                feedback.joint_names = list(names)
                feedback.desired = point
                with self._state_lock:
                    feedback.actual.positions = [
                        self._positions.get(name, 0.0) for name in names]
                goal_handle.publish_feedback(feedback)

            goal_tolerance = float(
                self.get_parameter('goal_tolerance').value)
            time_tolerance = float(
                self.get_parameter('goal_time_tolerance').value)
            final_positions = trajectory.points[-1].positions
            deadline = time.monotonic() + max(0.0, time_tolerance)

            while True:
                if goal_handle.is_cancel_requested:
                    goal_handle.canceled()
                    result.error_code = result.SUCCESSFUL
                    result.error_string = 'Trajectory canceled'
                    return result

                with self._state_lock:
                    errors = [
                        abs(self._positions.get(name, float('inf')) - target)
                        for name, target in zip(names, final_positions)
                    ]
                if errors and max(errors) <= goal_tolerance:
                    goal_handle.succeed()
                    result.error_code = result.SUCCESSFUL
                    result.error_string = 'SO-101 reached the target'
                    return result
                if time.monotonic() >= deadline:
                    goal_handle.abort()
                    result.error_code = result.GOAL_TOLERANCE_VIOLATED
                    result.error_string = (
                        'Target tolerance not reached; maximum joint error is '
                        f'{max(errors):.3f} rad')
                    return result
                time.sleep(0.05)
        finally:
            self._active_goal = None


def main(args=None):
    import rclpy

    rclpy.init(args=args)
    node = FollowJointTrajectoryBridge()
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
