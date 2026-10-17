import sys
from pathlib import Path

# 현재 터미널 위치가 아니라 이 파일의 위치를 기준으로 프로젝트 폴더를 찾습니다.
# convert.py: isaac_ros_so/isaac_jacobian_2/convert.py
# env_config_setting.py: isaac_ros_so/env_config_setting.py
project_dir = Path(__file__).resolve().parent.parent
if str(project_dir) not in sys.path:
    sys.path.insert(0, str(project_dir))

from env_config_setting import *

# 그리퍼 끝 TCP 위치 오프셋을 base 좌표계로 바꾼다.
def get_gripper_tcp_offset_b(ee_quat_b):
    tcp_offset_g = torch.tensor(
        GRIPPER_TCP_OFFSET,
        device=ee_quat_b.device,
        dtype=ee_quat_b.dtype,
    ).repeat(ee_quat_b.shape[0], 1)
    return quat_apply(ee_quat_b, tcp_offset_g)

# 그리퍼 TCP 기준으로 Jacobian을 다시 계산한다.
def get_gripper_tcp_jacobian(jacobian, tcp_offset_b):
    jacobian_tcp = jacobian.clone()
    jacobian_tcp[:, 0:3, :] = (
        jacobian[:, 0:3, :]
        - torch.bmm(skew_symmetric_matrix(tcp_offset_b), jacobian[:, 3:6, :])
    )
    return jacobian_tcp

# 시뮬레이션 집게값을 실제 로봇 집게값으로 바꾼다.
def sim_jaw_to_real_jaw(jaw_target):
    jaw_ratio = (jaw_target - CLOSE_JAW) / (OPEN_JAW - CLOSE_JAW)
    jaw_ratio = torch.clamp(jaw_ratio, 0.0, 1.0)
    return REAL_CLOSE_JAW + jaw_ratio * (REAL_OPEN_JAW - REAL_CLOSE_JAW)

# 실제 로봇 집게값을 시뮬레이션 집게값으로 바꾼다.
def real_jaw_to_sim_jaw(real_jaw):
    jaw_ratio = (real_jaw - REAL_CLOSE_JAW) / (REAL_OPEN_JAW - REAL_CLOSE_JAW)
    jaw_ratio = np.clip(jaw_ratio, 0.0, 1.0)
    return CLOSE_JAW + jaw_ratio * (OPEN_JAW - CLOSE_JAW)

# 시뮬레이션 팔 관절값을 실제 로봇 팔 명령값으로 바꾼다.
def sim_arm_to_real_arm(joint_pos_des):
    real_arm_target = joint_pos_des[0].clone()
    for joint_index, joint_name in enumerate(ARM_COMMAND_JOINT_NAMES):
        real_arm_target[joint_index] += ISAAC_TO_REAL_COMMAND_OFFSET[joint_name]
    return real_arm_target

# 실제 로봇 팔 관절값을 시뮬레이션 팔 관절값으로 바꾼다.
def real_arm_to_sim_arm(real_arm_pos):
    sim_arm_pos = real_arm_pos.copy()
    for joint_index, joint_name in enumerate(ARM_COMMAND_JOINT_NAMES):
        sim_arm_pos[joint_index] -= REAL_TO_ISAAC_STATE_OFFSET[joint_name]
    return sim_arm_pos

# 실제 로봇 전체 관절값을 시뮬레이션 전체 관절값으로 바꾼다.
def real_joint_pos_to_sim_joint_pos(real_joint_pos):
    sim_arm_pos = real_arm_to_sim_arm(real_joint_pos[:-1])
    sim_jaw_pos = real_jaw_to_sim_jaw(real_joint_pos[-1])
    return np.concatenate([sim_arm_pos, [sim_jaw_pos]])

# 시뮬레이션 목표값으로 실제 로봇에 보낼 전체 명령을 만든다.
def make_real_joint_command(joint_pos_des, jaw_target):
    real_arm_target = sim_arm_to_real_arm(joint_pos_des)
    real_jaw_target = sim_jaw_to_real_jaw(jaw_target)
    
    return torch.cat(
        [real_arm_target, real_jaw_target[0]],
        dim=0,
    ).detach().cpu().numpy()


def limit_real_joint_command(desired_command, current_positions):
    """실제 명령을 공통 관절 범위와 1회 최대 이동량 안으로 제한한다."""
    desired = np.asarray(desired_command, dtype=np.float64).reshape(-1)
    current = np.asarray(current_positions, dtype=np.float64).reshape(-1)
    expected_size = len(ROBOT_COMMAND_JOINT_NAMES)
    if desired.size != expected_size or current.size != expected_size:
        raise ValueError(
            f"관절값 개수가 잘못되었습니다: desired={desired.size}, "
            f"current={current.size}, expected={expected_size}"
        )
    if not np.all(np.isfinite(desired)) or not np.all(np.isfinite(current)):
        raise ValueError("관절 명령 또는 현재 관절값에 NaN/Inf가 있습니다.")

    lower = np.asarray(REAL_JOINT_LOWER_LIMITS_RAD, dtype=np.float64)
    upper = np.asarray(REAL_JOINT_UPPER_LIMITS_RAD, dtype=np.float64)
    if np.any(current < lower - 0.05) or np.any(current > upper + 0.05):
        raise RuntimeError(
            "현재 관절값이 안전 범위를 크게 벗어났습니다: "
            f"{current.tolist()}"
        )

    limited_desired = np.clip(desired, lower, upper)
    requested_delta = limited_desired - current
    command_delta = np.clip(
        requested_delta,
        -MAX_REAL_COMMAND_STEP_RAD,
        MAX_REAL_COMMAND_STEP_RAD,
    )
    safe_command = np.clip(current + command_delta, lower, upper)
    was_limited = bool(
        np.any(np.abs(desired - limited_desired) > 1.0e-9)
        or np.any(np.abs(requested_delta - command_delta) > 1.0e-9)
    )
    return safe_command, requested_delta, command_delta, was_limited

# 실제 로봇을 초기 자세로 보내는 명령을 만든다.
def make_real_reset_joint_command():
    return [
        REAL_COMMAND_ZERO_JOINT_POS[joint_name]
        for joint_name in ROBOT_COMMAND_JOINT_NAMES
    ]
