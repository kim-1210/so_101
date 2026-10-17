from lib_imports_2 import *

class SetRobotPose(Node):
    def __init__(self):
        super().__init__("SetRobotPose")

        self.pub_robot_pose = self.create_publisher(
            Float64MultiArray,
            "/follower/joint_commands",
            10
        )

    def set_pose_callback(self, joint_positions):
        msg = Float64MultiArray()
        msg.data = joint_positions
        self.pub_robot_pose.publish(msg)
