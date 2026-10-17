import time

from convert import *

# 실제 로봇을 작은 관절 간격으로 안전하게 초기 자세까지 보낸다.
def hold_real_robot_reset(
    sim,
    scene,
    robot,
    robot_entity_cfg,
    jaw_id,
    jaw_target,
    joint_pos_des,
    robot_joint_pub_node,
    robot_joint_sub_node,
    sim_dt,
):
    reset_command = np.asarray(make_real_reset_joint_command(), dtype=np.float64)
    wait_until = time.monotonic() + RESET_TIMEOUT_SECONDS
    pending_command = None
    print(
        "[INFO] 실제 로봇을 안전한 간격으로 reset 자세까지 이동합니다.",
        flush=True,
    )

    while simulation_app.is_running() and time.monotonic() < wait_until:
        rclpy.spin_once(robot_joint_sub_node, timeout_sec=min(sim_dt, 0.02))
        current_positions = robot_joint_sub_node.get_joint_pos(
            ROBOT_COMMAND_JOINT_NAMES
        )
        if current_positions is None:
            continue

        current_positions = np.asarray(current_positions, dtype=np.float64)
        if pending_command is None or np.all(
            np.abs(current_positions - pending_command)
            <= MAX_REAL_SYNC_TOLERANCE_RAD
        ):
            if np.all(
                np.abs(current_positions - reset_command)
                <= MAX_REAL_SYNC_TOLERANCE_RAD
            ):
                print("[INFO] 실제 로봇 reset 자세 도착", flush=True)
                return
            pending_command, _, _, _ = limit_real_joint_command(
                reset_command,
                current_positions,
            )

        robot_joint_pub_node.set_pose_callback(pending_command.tolist())
        robot.set_joint_position_target(
            joint_pos_des,
            joint_ids=robot_entity_cfg.joint_ids,
        )
        robot.set_joint_position_target(jaw_target, joint_ids=[jaw_id])
        scene.write_data_to_sim()
        sim.step()
        scene.update(sim_dt)

        time.sleep(sim_dt)

    raise RuntimeError(
        f"실제 로봇이 {RESET_TIMEOUT_SECONDS:.1f}초 안에 reset 자세에 "
        "도착하지 못했습니다. 모터 상태와 장애물을 확인하세요."
    )

# 로봇 관절을 초기 자세와 속도로 만들 값을 준비한다.
def make_reset_joint_state(robot):
    joint_pos = robot.data.default_joint_pos.clone()
    joint_vel = torch.zeros_like(robot.data.default_joint_vel)

    for joint_name, value in RESET_JOINT_POS.items():
        joint_id = robot.data.joint_names.index(joint_name)
        joint_pos[:, joint_id] = value

    return joint_pos, joint_vel

# Jacobian IK로 목표 TCP 위치에 맞는 팔 관절값을 구한다.
def move_jacobian(robot, ee_jacobi_idx, robot_entity_cfg, diff_ik_controller):
    jacobian = robot.root_physx_view.get_jacobians()[
        :, ee_jacobi_idx, :, robot_entity_cfg.joint_ids
    ]
    ee_pose_w = robot.data.body_pose_w[:, robot_entity_cfg.body_ids[0]]
    root_pose_w = robot.data.root_pose_w
    joint_pos = robot.data.joint_pos[:, robot_entity_cfg.joint_ids]
    ee_pos_b, ee_quat_b = subtract_frame_transforms(
        root_pose_w[:, 0:3],
        root_pose_w[:, 3:7],
        ee_pose_w[:, 0:3],
        ee_pose_w[:, 3:7],
    )
    tcp_offset_b = get_gripper_tcp_offset_b(ee_quat_b)
    tcp_pos_b = ee_pos_b + tcp_offset_b
    jacobian_tcp = get_gripper_tcp_jacobian(jacobian, tcp_offset_b)
    candidate = diff_ik_controller.compute(
        tcp_pos_b,
        ee_quat_b,
        jacobian_tcp,
        joint_pos,
    )
    delta = torch.clamp(
        candidate - joint_pos,
        min=-MAX_SIM_IK_STEP_RAD,
        max=MAX_SIM_IK_STEP_RAD,
    )
    candidate = joint_pos + delta
    soft_limits = robot.data.soft_joint_pos_limits[:, robot_entity_cfg.joint_ids]
    return torch.maximum(
        torch.minimum(candidate, soft_limits[..., 1]),
        soft_limits[..., 0],
    )

# 집게를 열거나 닫을 목표값을 만든다.
def get_jaw_goal(jaw_target, is_open_grasp=False):
    if is_open_grasp:
        jaw_value = OPEN_JAW
    else:
        jaw_value = CLOSE_JAW
    return torch.full_like(jaw_target, jaw_value)

# 집게가 목표값까지 천천히 움직이도록 다음 값을 계산한다.
def move_grasp(jaw_target, sim_dt, is_open_grasp=False):
    jaw_goal = get_jaw_goal(jaw_target, is_open_grasp)
    max_delta = JAW_SPEED * sim_dt
    return jaw_target + torch.clamp(
        jaw_goal - jaw_target,
        min=-max_delta,
        max=max_delta,
    )

def load_IK(sim: sim_utils.SimulationContext, scene: InteractiveScene):
    diff_ik_cfg = DifferentialIKControllerCfg(
        command_type="position",
        use_relative_mode=False,
        ik_method="dls",
        ik_params={"lambda_val": IK_DAMPING},
    )
    return DifferentialIKController(
        diff_ik_cfg,
        num_envs=scene.num_envs,
        device=sim.device,
    )

def get_ee_jacobi_idx(robot, robot_entity_cfg):
    if robot.is_fixed_base:
        return robot_entity_cfg.body_ids[0] - 1
    return robot_entity_cfg.body_ids[0]

# 집게가 목표 열림/닫힘 값에 도착했는지 확인한다.
def is_jaw_reached(jaw_target, is_open_grasp=False):
    jaw_goal = get_jaw_goal(jaw_target, is_open_grasp)
    return torch.all(torch.abs(jaw_goal - jaw_target) < 1.0e-4)

def set_pick_place_target(state, ik_commands, diff_ik_controller):
    command_num = state["command_num"]
    target_name_by_command = {
        1: "grasp_target",
        3: "lift_target",
        4: "transfer_target",
        5: "place_target",
    }
    target_name = target_name_by_command.get(command_num)
    if target_name is None:
        return

    target = torch.as_tensor(
        state[target_name],
        device=ik_commands.device,
        dtype=ik_commands.dtype,
    ).repeat(ik_commands.shape[0], 1)
    ik_commands[:] = target

    diff_ik_controller.set_command(ik_commands, ee_quat=state["ee_quat_b"])
    state["is_target_position_set"] = True
    print(
        f"[STEP {command_num}] {target_name}: "
        f"{np.round(state[target_name], 4).tolist()} m",
        flush=True,
    )

def is_command_done(command_num, jaw_target, tcp_pos_w, goal_pos_w):
    if command_num == 2:
        return is_jaw_reached(jaw_target, is_open_grasp=False)
    if command_num == 6:
        return is_jaw_reached(jaw_target, is_open_grasp=True)

    dist = torch.norm(tcp_pos_w - goal_pos_w, dim=1)
    return torch.all(dist < COMMAND_THRESHOLD[command_num])

def run_pick_place_step(
    state,
    robot,
    robot_entity_cfg,
    ee_jacobi_idx,
    diff_ik_controller,
    ik_commands,
    jaw_target,
    sim_dt,
):
    command_num = state["command_num"]
    state["stage_step_count"] += 1

    if command_num in (1, 3, 4, 5) and not state["is_target_position_set"]:
        set_pick_place_target(state, ik_commands, diff_ik_controller)

    if command_num in (0, 1, 3, 4, 5):
        state["joint_pos_des"] = move_jacobian(
            robot=robot,
            ee_jacobi_idx=ee_jacobi_idx,
            robot_entity_cfg=robot_entity_cfg,
            diff_ik_controller=diff_ik_controller,
        )
    elif command_num == 2:
        jaw_target = move_grasp(jaw_target, sim_dt, is_open_grasp=False)
    elif command_num == 6:
        jaw_target = move_grasp(jaw_target, sim_dt, is_open_grasp=True)

    if is_command_done(
        command_num,
        jaw_target,
        state["tcp_pos_w"],
        state["goal_pos_w"],
    ):
        state["reach_count"] += 1
    else:
        state["reach_count"] = 0

    if command_num not in (2, 6) and state["stage_step_count"] % 100 == 0:
        distance_mm = float(
            torch.norm(state["tcp_pos_w"] - state["goal_pos_w"], dim=1)[0]
            * 1000.0
        )
        joint_pos = robot.data.joint_pos[0, robot_entity_cfg.joint_ids]
        soft_limits = robot.data.soft_joint_pos_limits[0, robot_entity_cfg.joint_ids]
        limit_margin = torch.minimum(
            joint_pos - soft_limits[:, 0],
            soft_limits[:, 1] - joint_pos,
        )
        print(
            f"[IK] step={command_num}, TCP 오차={distance_mm:.1f} mm, "
            f"관절={np.round(joint_pos.detach().cpu().numpy(), 3).tolist()}, "
            f"최소 limit 여유={float(torch.min(limit_margin)):.3f} rad",
            flush=True,
        )

    if state["stage_step_count"] > 3000:
        distance_mm = float(
            torch.norm(state["tcp_pos_w"] - state["goal_pos_w"], dim=1)[0]
            * 1000.0
        )
        raise RuntimeError(
            f"IK가 step {command_num} 목표에 수렴하지 못했습니다: "
            f"TCP 오차={distance_mm:.1f} mm"
        )

    if state["reach_count"] > 20:
        completed_command = state["command_num"]
        state["command_num"] += 1
        state["is_target_position_set"] = False
        state["reach_count"] = 0
        state["stage_step_count"] = 0
        print(f"[STEP {completed_command}] 완료", flush=True)

    if state["command_num"] == 7:
        state["needs_reset"] = True
        print("[TASK] 집기/놓기 1회 완료. 다음 목표를 다시 읽습니다.", flush=True)

    return jaw_target
