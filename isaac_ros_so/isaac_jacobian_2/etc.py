from control import *
import time

from ros_get_cube_pose import *
from ros_get_robot_pose import *
from ros_set_robot_pose import *


def make_real_sync_state():
    return {
        "pending_command": None,
        "pending_since": None,
        "last_publish_time": 0.0,
        "last_command_count": -1,
    }


def reset_real_sync_state(sync_state):
    sync_state.update(make_real_sync_state())


def effective_sync_threshold():
    requested = float(args_cli.sim_to_real_diff)
    if not np.isfinite(requested) or requested <= 0.0:
        raise ValueError("--sim_to_real_diff는 0보다 큰 rad 값이어야 합니다.")
    return min(requested, MAX_REAL_SYNC_TOLERANCE_RAD)


def validate_target_position(position, label):
    if position is None:
        return None
    value = np.asarray(position, dtype=np.float64).reshape(-1)
    if value.size != 3 or not np.all(np.isfinite(value)):
        print(f"[WARN] {label} 좌표가 올바르지 않습니다: {position}", flush=True)
        return None
    if np.linalg.norm(value) > MAX_TARGET_DISTANCE_M:
        print(
            f"[WARN] {label} 좌표가 작업 거리 밖입니다: {value.tolist()} "
            f"(최대 {MAX_TARGET_DISTANCE_M:.2f} m)",
            flush=True,
        )
        return None
    if not MIN_TARGET_Z_M <= value[2] <= MAX_TARGET_Z_M:
        print(
            f"[WARN] {label} Z가 안전 범위 밖입니다: z={value[2]:.3f} m",
            flush=True,
        )
        return None
    return value

# 큐브를 지정 위치나 랜덤 위치로 다시 배치한다.
def reset_cube(scene, position=None):
    if position is None:
        origins = scene.env_origins
        pos = origins + torch.stack(
            [
                torch.rand(scene.num_envs, device=scene.device) * 0.1 - 0.05,
                torch.rand(scene.num_envs, device=scene.device) * 0.1 + 0.30,
                torch.ones(scene.num_envs, device=scene.device) * 1.08,
            ],
            dim=-1,
        )
    else:
        pos = position

    quat = torch.tensor(
        [1., 0., 0., 0.],
        device=scene.device,
    ).repeat(scene.num_envs, 1)

    scene["cube"].write_root_pose_to_sim(torch.cat([pos, quat], dim=-1))
    
# 실제 로봇과 시뮬레이션 관절값 차이가 충분히 작은지 확인한다.
def is_real_robot_synced_with_sim(
    robot_joint_sub_node,
    sim_arm_joint_pos,
    jaw_target,
    threshold,
):
    rclpy.spin_once(robot_joint_sub_node, timeout_sec=0.0)
    real_joint_pos = robot_joint_sub_node.get_joint_pos(
        ROBOT_COMMAND_JOINT_NAMES
    )
    if real_joint_pos is None:
        return False

    real_joint_pos = np.array(real_joint_pos, dtype=np.float32)
    real_as_sim_joint_pos = real_joint_pos_to_sim_joint_pos(real_joint_pos)
    # sim_arm_joint_pos = sim_arm_joint_pos[0].detach().cpu().numpy()
    sim_jaw_pos = jaw_target[0].detach().cpu().numpy()
    # sim_jaw_pos = jaw_target
    sim_target_joint_pos = np.concatenate([sim_arm_joint_pos, sim_jaw_pos])
    signed_diff = real_as_sim_joint_pos - sim_target_joint_pos
    abs_diff = np.abs(signed_diff)

    is_synced = not np.any(abs_diff > threshold)
    now = time.monotonic()
    last_print_time = getattr(
        is_real_robot_synced_with_sim,
        "_last_print_time",
        0.0,
    )
    if not is_synced and now - last_print_time >= SYNC_PRINT_INTERVAL_SECONDS:
        # print(f"Sync diff: {abs_diff} | signed: {signed_diff}", flush=True)
        is_real_robot_synced_with_sim._last_print_time = now

    return is_synced

def set_robot_joint_cfg():
    robot_entity_cfg = SceneEntityCfg(
        "robot",
        joint_names=[
            "Rotation", "Pitch", 
            "Elbow", "Wrist_Pitch", 
            "Wrist_Roll"
        ],
        body_names=["gripper"]
    )
    return robot_entity_cfg

def setup_markers():
    frame_marker_cfg = FRAME_MARKER_CFG.copy()
    frame_marker_cfg.markers["frame"].scale = (0.1, 0.1, 0.1)
    ee_marker = VisualizationMarkers(
        frame_marker_cfg.replace(prim_path="/Visuals/ee_current")
    )
    goal_marker = VisualizationMarkers(
        frame_marker_cfg.replace(prim_path="/Visuals/ee_goal")
    )
    return ee_marker, goal_marker

def sync_real_robot(
    robot,
    robot_entity_cfg,
    jaw_target,
    robot_joint_pub_node,
    robot_joint_sub_node,
    count,
    sync_state,
):
    if not args_cli.sim_to_real:
        return True

    threshold = effective_sync_threshold()
    rclpy.spin_once(robot_joint_sub_node, timeout_sec=0.01)
    current_positions = robot_joint_sub_node.get_joint_pos(
        ROBOT_COMMAND_JOINT_NAMES
    )
    if current_positions is None:
        return False
    current_positions = np.asarray(current_positions, dtype=np.float64)
    now = time.monotonic()

    pending_command = sync_state["pending_command"]
    if pending_command is not None:
        errors = np.abs(current_positions - pending_command)
        if np.all(errors <= threshold):
            sync_state["pending_command"] = None
            sync_state["pending_since"] = None
            return True

        if now - sync_state["pending_since"] > REAL_COMMAND_TIMEOUT_SECONDS:
            worst_index = int(np.argmax(errors))
            raise RuntimeError(
                f"실제 로봇 추종 시간 초과: "
                f"{ROBOT_COMMAND_JOINT_NAMES[worst_index]} 오차 "
                f"{errors[worst_index]:.4f} rad. 장애물/토크/모터 상태를 확인하세요."
            )

        if now - sync_state["last_publish_time"] >= REAL_COMMAND_REPUBLISH_SECONDS:
            robot_joint_pub_node.set_pose_callback(pending_command.tolist())
            sync_state["last_publish_time"] = now
        time.sleep(0.01)
        return False

    if count % REAL_COMMAND_INTERVAL_SIM_STEPS != 0:
        return True
    if sync_state["last_command_count"] == count:
        return True

    desired_command = make_real_joint_command(
        robot.data.joint_pos[:, robot_entity_cfg.joint_ids],
        jaw_target,
    )
    safe_command, requested_delta, command_delta, was_limited = (
        limit_real_joint_command(desired_command, current_positions)
    )
    sync_state["last_command_count"] = count

    if np.all(np.abs(current_positions - safe_command) <= threshold):
        return True

    robot_joint_pub_node.set_pose_callback(safe_command.tolist())
    sync_state["pending_command"] = safe_command
    sync_state["pending_since"] = now
    sync_state["last_publish_time"] = now
    if was_limited:
        print(
            "[LIMIT] 실제 관절 명령 제한: "
            f"요청 최대={np.max(np.abs(requested_delta)):.3f} rad, "
            f"전송 최대={np.max(np.abs(command_delta)):.3f} rad",
            flush=True,
        )
    return False

def reset_task(
    sim,
    scene,
    robot,
    cube,
    cube_node,
    robot_joint_pub_node,
    robot_joint_sub_node,
    robot_entity_cfg,
    jaw_id,
    jaw_target,
    ik_commands,
    diff_ik_controller,
    sim_dt,
):
    joint_pos, joint_vel = make_reset_joint_state(robot)
    robot.write_joint_state_to_sim(joint_pos, joint_vel)
    robot.set_joint_position_target(joint_pos)
    robot.reset()
    jaw_target[:] = OPEN_JAW

    basket_position = cube_node.get_basket_position()
    if basket_position is not None:
        basket_position = np.asarray(basket_position, dtype=np.float64).copy()
        if basket_position.size == 3:
            basket_position.reshape(-1)[2] = FIXED_BASKET_Z_M
    basket_position = validate_target_position(basket_position, "basket")
    if basket_position is None:
        now = time.monotonic()
        if now - getattr(reset_task, "_last_wait_log", 0.0) >= 1.0:
            print("[WAIT] /basket 좌표를 기다리는 중...", flush=True)
            reset_task._last_wait_log = now
        rclpy.spin_once(cube_node, timeout_sec=0.1)
        return None

    object_position = cube_node.get_position()
    if object_position is not None:
        object_position = np.asarray(object_position, dtype=np.float64).copy()
        if object_position.size == 3:
            object_position.reshape(-1)[2] = FIXED_OBJECT_Z_M
    object_position = validate_target_position(object_position, "object")
    if object_position is None:
        now = time.monotonic()
        if now - getattr(reset_task, "_last_wait_log", 0.0) >= 1.0:
            print("[WAIT] /gripper_targets 좌표를 기다리는 중...", flush=True)
            reset_task._last_wait_log = now
        rclpy.spin_once(cube_node, timeout_sec=0.1)
        return None

    approach_target = object_position.copy()
    approach_target[2] += PICK_APPROACH_CLEARANCE_M
    grasp_target = object_position.copy()
    grasp_target[2] += PICK_GRASP_CLEARANCE_M
    lift_target = object_position.copy()
    lift_target[2] = FIXED_BASKET_Z_M
    transfer_target = basket_position.copy()
    transfer_target[2] = FIXED_BASKET_Z_M
    place_target = basket_position.copy()
    place_target[2] = FIXED_BASKET_Z_M

    planned_targets = {
        "approach": approach_target,
        "grasp": grasp_target,
        "lift": lift_target,
        "transfer": transfer_target,
        "place": place_target,
    }
    for label, target in planned_targets.items():
        if validate_target_position(target, label) is None:
            return None

    print(
        "[TARGET] "
        f"object={np.round(object_position, 4).tolist()} m, "
        f"basket={np.round(basket_position, 4).tolist()} m",
        flush=True,
    )
    print(
        "[PLAN] "
        + ", ".join(
            f"{name}={np.round(target, 4).tolist()}"
            for name, target in planned_targets.items()
        ),
        flush=True,
    )

    cube_pos_b = torch.as_tensor(
        object_position,
        device=scene.device,
        dtype=robot.data.root_pose_w.dtype,
    ).repeat(scene.num_envs, 1)
    cube_quat_b = torch.tensor(
        [1.0, 0.0, 0.0, 0.0],
        device=scene.device,
    ).repeat(scene.num_envs, 1)
    cube_pos_w, _ = combine_frame_transforms(
        robot.data.root_pose_w[:, 0:3],
        robot.data.root_pose_w[:, 3:7],
        cube_pos_b,
        cube_quat_b,
    )
    reset_cube(scene, cube_pos_w)

    joint_pos_des = joint_pos[:, robot_entity_cfg.joint_ids].clone()
    if args_cli.sim_to_real:
        hold_real_robot_reset(
            sim=sim,
            scene=scene,
            robot=robot,
            robot_entity_cfg=robot_entity_cfg,
            jaw_id=jaw_id,
            jaw_target=jaw_target,
            joint_pos_des=joint_pos_des,
            robot_joint_pub_node=robot_joint_pub_node,
            robot_joint_sub_node=robot_joint_sub_node,
            sim_dt=sim_dt,
        )

    diff_ik_controller.reset()
    ee_pose_w = robot.data.body_pose_w[:, robot_entity_cfg.body_ids[0]]
    root_pose_w = robot.data.root_pose_w
    _, ee_quat_b = subtract_frame_transforms(
        root_pose_w[:, 0:3],
        root_pose_w[:, 3:7],
        ee_pose_w[:, 0:3],
        ee_pose_w[:, 3:7],
    )

    ik_commands[:] = torch.as_tensor(
        approach_target,
        device=scene.device,
        dtype=ik_commands.dtype,
    ).repeat(scene.num_envs, 1)
    diff_ik_controller.set_command(ik_commands, ee_quat=ee_quat_b)

    return {
        "object_position": object_position,
        "basket_position": basket_position,
        "approach_target": approach_target,
        "grasp_target": grasp_target,
        "lift_target": lift_target,
        "transfer_target": transfer_target,
        "place_target": place_target,
        "command_num": 0,
        "count": 0,
        "ee_quat_b": ee_quat_b,
        "is_target_position_set": False,
        "joint_pos_des": joint_pos_des,
        "reach_count": 0,
        "stage_step_count": 0,
        "needs_reset": False,
    }

def step_sim(sim, scene, robot, robot_entity_cfg, jaw_id, state, jaw_target, sim_dt):
    robot.set_joint_position_target(
        state["joint_pos_des"],
        joint_ids=robot_entity_cfg.joint_ids,
    )
    robot.set_joint_position_target(jaw_target, joint_ids=[jaw_id])
    scene.write_data_to_sim()
    sim.step()
    state["count"] += 1
    scene.update(sim_dt)

def update_markers(
    scene,
    robot,
    robot_entity_cfg,
    ik_commands,
    ee_marker,
    goal_marker,
):
    ee_pose_w = robot.data.body_state_w[:, robot_entity_cfg.body_ids[0], 0:7]
    tcp_offset_g = torch.tensor(
        GRIPPER_TCP_OFFSET,
        device=robot.device,
        dtype=ee_pose_w.dtype,
    ).repeat(scene.num_envs, 1)
    tcp_quat_g = torch.tensor(
        [1.0, 0.0, 0.0, 0.0],
        device=robot.device,
        dtype=ee_pose_w.dtype,
    ).repeat(scene.num_envs, 1)

    tcp_pos_w, tcp_quat_w = combine_frame_transforms(
        ee_pose_w[:, 0:3],
        ee_pose_w[:, 3:7],
        tcp_offset_g,
        tcp_quat_g,
    )
    goal_pos_w, goal_quat_w = combine_frame_transforms(
        robot.data.root_pose_w[:, 0:3],
        robot.data.root_pose_w[:, 3:7],
        ik_commands,
        tcp_quat_g,
    )

    ee_marker.visualize(tcp_pos_w, tcp_quat_w)
    goal_marker.visualize(goal_pos_w, goal_quat_w)
    return tcp_pos_w, goal_pos_w
