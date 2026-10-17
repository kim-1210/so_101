from lib_imports_2 import *

class GetRobotPose(Node):
    def __init__(self):
        super().__init__("GetRobotPose")

        self.sub_robot_pose = self.create_subscription(
            msg_type=JointState,
            topic="/follower/joint_states",
            callback=self.sub_pose_callback,
            qos_profile=1,
        )
        self.joint_positions= None

    def sub_pose_callback(self, msg):
        self.joint_positions = dict(
            zip(msg.name, msg.position)
        )
        
    def get_joint_pos(self, joint_names=None):
        if self.joint_positions is None:
            return None
        if joint_names is None:
            return list(self.joint_positions.values())
        if not all(name in self.joint_positions for name in joint_names):
            return None
        return [self.joint_positions[name] for name in joint_names]
    
# if __name__ == "__main__":
#     rclpy.init()
#     node = GetRobotPose()
#     cnt = 0
#     while cnt < 2:
#         rclpy.spin_once(node)
#         print(f"pos: {node.get_joint_pos()}")
#         cnt+=1

#     node.destroy_node()
#     rclpy.shutdown()
