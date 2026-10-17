from lib_imports import *

@configclass
class TableTopSceneCfg(InteractiveSceneCfg):
    ground = AssetBaseCfg(
        prim_path="/World/Ground",
        spawn=GroundPlaneCfg()
    )

    dome_light = AssetBaseCfg(
        prim_path="/World/Light",
        spawn=sim_utils.DomeLightCfg(intensity=3000.0)
    )

    table = AssetBaseCfg(
        prim_path="{ENV_REGEX_NS}/Table",
        spawn=UsdFileCfg(
            usd_path=f"{ISAAC_NUCLEUS_DIR}/Props/Mounts/SeattleLabTable/table_instanceable.usd"
        ),
        init_state=AssetBaseCfg.InitialStateCfg(pos=(0.0, 0.0, 1.05)),
    )

    cube = RigidObjectCfg(
        prim_path="{ENV_REGEX_NS}/Cube",
        spawn=sim_utils.CuboidCfg(
            size=(0.03, 0.03, 0.03),
            rigid_props=sim_utils.RigidBodyPropertiesCfg(),
            collision_props=sim_utils.CollisionPropertiesCfg(
                contact_offset=0.002,
                rest_offset=0.0
            ),
            physics_material=RigidBodyMaterialCfg(
                static_friction=1.2,
                dynamic_friction=1.0,
                restitution=0.0,
                friction_combine_mode="max",
            ),
            visual_material=sim_utils.PreviewSurfaceCfg(diffuse_color=(0.0, 1.0, 0.0)),
        ),
        init_state=RigidObjectCfg.InitialStateCfg(
            pos=(0.05, 0.35, 1.08),
            rot=(1., 0., 0., 0.),
        ),
    )

    if args_cli.robot == "so_arm":
        robot = So_Arm_CFG.replace(
            prim_path="{ENV_REGEX_NS}/Robot",
            init_state=ArticulationCfg.InitialStateCfg(
                pos=(0.0, 0.6, 1.06),
                joint_pos={
                    "Rotation": 0.0084,
                    "Pitch": -0.602,
                    "Elbow": -0.2631,
                    "Wrist_Pitch": 0.639,
                    "Wrist_Roll": -2.7,
                    "Jaw": 0.1081,
                },
            ),
        )
    else:
        raise ValueError("fail")

DEVICE = torch.device("cuda" if torch.cuda.is_available() else "cpu")
# TCP 위치 목표의 완료 허용 오차(m)다. 예전 0.04는 40 mm라서
# 목표에 도착하기 전에 다음 동작으로 넘어가는 원인이 되었다.
COMMAND_THRESHOLD = {
    0: 0.006,
    1: 0.005,
    2: 0.005,
    3: 0.008,
    4: 0.012,
    5: 0.012,
    6: 0.005,
}
# ROS URDF의 gripper -> gripper_tcp 고정 변환과 동일한 값이다.
# so101_ros/src/so_arm_description/urdf/so_arm.urdf.xacro의
# gripper_tcp_joint origin과 한 값만 관리해야 sim/real TCP가 일치한다.
GRIPPER_TCP_OFFSET = (-0.002, -0.000218, -0.09)
ROBOT_COMMAND_JOINT_NAMES = [
    "Rotation",
    "Pitch",
    "Elbow",
    "Wrist_Pitch",
    "Wrist_Roll",
    "Jaw",
]
ARM_COMMAND_JOINT_NAMES = ROBOT_COMMAND_JOINT_NAMES[:-1]
RESET_JOINT_POS = {
    "Rotation": 0.0,
    "Pitch": -0.0,
    "Elbow": -0.0,
    "Wrist_Pitch": 0.0,
    "Wrist_Roll": 1.57,
    "Jaw": 0.8,
}
OPEN_JAW = 0.8
CLOSE_JAW = 0.0
REAL_OPEN_JAW = OPEN_JAW
REAL_CLOSE_JAW = CLOSE_JAW
REAL_COMMAND_ZERO_JOINT_POS = {
    "Rotation": 0.0,
    "Pitch": -0.0,
    "Elbow": -0.0,
    "Wrist_Pitch": 0.0,
    "Wrist_Roll": 1.57,
    # reset 때 시뮬레이션과 실제 로봇 모두 집게를 연 상태로 맞춘다.
    "Jaw": REAL_OPEN_JAW,
}

# /follower/hardware가 zero_offset_rad를 읽기/쓰기 양쪽에 적용하므로
# JointState와 JointCommand는 이미 ROS URDF와 같은 관절 좌표계를 사용한다.
# 예전에 실측한 별도 state 영점을 여기서 다시 빼면 영점 보정이 중복된다.
# 특히 Wrist_Roll은 1.57 rad가 한 번 더 더해지는 문제가 생긴다.
REAL_STATE_ZERO_JOINT_POS = {
    "Rotation": RESET_JOINT_POS["Rotation"],
    "Pitch": RESET_JOINT_POS["Pitch"],
    "Elbow": RESET_JOINT_POS["Elbow"],
    "Wrist_Pitch": RESET_JOINT_POS["Wrist_Pitch"],
    "Wrist_Roll": RESET_JOINT_POS["Wrist_Roll"],
    "Jaw": 0.0,
}
ISAAC_TO_REAL_COMMAND_OFFSET = {
    joint_name: REAL_COMMAND_ZERO_JOINT_POS[joint_name] - RESET_JOINT_POS[joint_name]
    for joint_name in ARM_COMMAND_JOINT_NAMES
}
REAL_TO_ISAAC_STATE_OFFSET = {
    joint_name: REAL_STATE_ZERO_JOINT_POS[joint_name] - RESET_JOINT_POS[joint_name]
    for joint_name in ARM_COMMAND_JOINT_NAMES
}
JAW_SPEED = 0.7  # rad/s
SYNC_PRINT_INTERVAL_SECONDS = 1.0
MAX_STEP = 1200

# IK와 실제 로봇 추종에 사용하는 안전 설정이다.
IK_DAMPING = 0.01
MAX_SIM_IK_STEP_RAD = 0.08
MAX_REAL_COMMAND_STEP_RAD = 0.08
MAX_REAL_SYNC_TOLERANCE_RAD = 0.02
REAL_COMMAND_INTERVAL_SIM_STEPS = 10
REAL_COMMAND_REPUBLISH_SECONDS = 0.20
REAL_COMMAND_TIMEOUT_SECONDS = 8.0
RESET_TIMEOUT_SECONDS = 30.0

# Isaac USD, ROS URDF, follower 모터 범위가 모두 겹치는 안전 관절 범위다.
# 순서는 ROBOT_COMMAND_JOINT_NAMES와 같다.
REAL_JOINT_LOWER_LIMITS_RAD = (
    -1.794,
    -1.745,
    -1.714,
    -1.658,
    -2.793,
    -0.163,
)
REAL_JOINT_UPPER_LIMITS_RAD = (
    1.719,
    1.685,
    1.571,
    1.658,
    2.793,
    1.168,
)

# XY는 카메라 값을 사용하지만 Z는 테이블/바구니의 고정 높이를 사용한다.
FIXED_OBJECT_Z_M = 0.01
FIXED_BASKET_Z_M = 0.18

# 고정 Z를 기준으로 접근/파지 높이를 만든다.
PICK_APPROACH_CLEARANCE_M = 0.08
PICK_GRASP_CLEARANCE_M = 0.005
MIN_TARGET_Z_M = 0.0
MAX_TARGET_Z_M = 0.35
MAX_TARGET_DISTANCE_M = 0.42
