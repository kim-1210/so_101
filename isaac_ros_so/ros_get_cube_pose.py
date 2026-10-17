from lib_imports_2 import *

class GetCubePose(Node):
    def __init__(self):
        super().__init__("GetCubePose")

        self.sub_cube_pose = self.create_subscription(
            msg_type=PoseArray,
            topic="/gripper_targets",
            callback=self.sub_pose_callback,
            qos_profile=1,
        )

        self.sub_cube_pose = self.create_subscription(
            msg_type=Pose,
            topic="/basket",
            callback=self.sub_basket_callback,
            qos_profile=1,
        )

        self.poses = None
        self.basket_pose = None

    def sub_basket_callback(self, msg):
        self.basket_pose = msg.position

    def sub_pose_callback(self, msg):
        if msg.header.frame_id not in ("", "base"):
            self.get_logger().error(
                f"/gripper_targets frame은 base여야 합니다: {msg.header.frame_id}"
            )
            return
        self.poses = list(msg.poses) if msg.poses else None

    def get_basket_position(self):
        if self.basket_pose is None:
            return None

        return [
            self.basket_pose.x,
            self.basket_pose.y,
            self.basket_pose.z
        ]

    # base에서 가까운것
    def get_position(self):
        min_dist = 9999999
        index = -1
        min_x = None
        min_y = None
        min_z = None

        if self.poses is None:
            return None
        
        for idx, pose in enumerate(self.poses):
            value = np.sqrt(pose.position.x ** 2 + pose.position.y ** 2 + pose.position.z ** 2)
            if min_dist > value:
                min_dist = value
                min_x = pose.position.x
                min_y = pose.position.y
                min_z = pose.position.z
                index = idx

        del self.poses[index]
        if len(self.poses) == 0:
            self.poses=None 
            
        return [min_x, min_y, min_z]
