"""
isaaclab -p ./isaac_ros_so/isaac_jacobian.py \
--x 0.227 \
--y -0.203 \
--z 0.0

isaaclab_4 -p ./isaac_ros_so/isaac_jacobian.py \
  --kit_args="--/log/level=error --/log/fileLogLevel=error --/log/outputStreamLevel=error" \
  --num_envs 1 \
  --sim_to_real True \
  --sim_to_real_diff 0.05

ros2 topic pub --once /follower/joint_commands std_msgs/msg/Float64MultiArray "{data: [-1.0, -1.0, 0.0, 0.0, 0.0, 0.0]}"
"""

from env_config_setting import *
from ros_get_cube_pose import *
from ros_get_robot_pose import *
from ros_set_robot_pose import *
import time

INITIAL_REAL_ROBOT_WAIT_SECONDS = 5.0

# 로봇 관절을 초기 자세와 속도로 만들 값을 준비한다.
def make_reset_joint_state(robot):
    joint_pos = robot.data.default_joint_pos.clone()
    joint_vel = torch.zeros_like(robot.data.default_joint_vel)

    for joint_name, value in RESET_JOINT_POS.items():
        joint_id = robot.data.joint_names.index(joint_name)
        joint_pos[:, joint_id] = value

    return joint_pos, joint_vel

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

# Jacobian IK로 목표 TCP 위치에 맞는 팔 관절값을 구한다.
def move_jacobian(
    robot,
    ee_jacobi_idx,
    robot_entity_cfg,
    diff_ik_controller,
):
    # obtain quantities from simulation
    jacobian = robot.root_physx_view.get_jacobians()[:, ee_jacobi_idx, :, robot_entity_cfg.joint_ids]
    ee_pose_w = robot.data.body_pose_w[:, robot_entity_cfg.body_ids[0]]
    root_pose_w = robot.data.root_pose_w
    joint_pos = robot.data.joint_pos[:, robot_entity_cfg.joint_ids]
    # 월드좌표 ee를 base기준 좌표로 바꾸기 위해
    ee_pos_b, ee_quat_b = subtract_frame_transforms(
        root_pose_w[:, 0:3], root_pose_w[:, 3:7], ee_pose_w[:, 0:3], ee_pose_w[:, 3:7]
    )
    # offset설정
    tcp_offset_b = get_gripper_tcp_offset_b(ee_quat_b)
    tcp_pos_b = ee_pos_b + tcp_offset_b
    jacobian_tcp = get_gripper_tcp_jacobian(jacobian, tcp_offset_b)
    # ik실행
    joint_pos_des = diff_ik_controller.compute(tcp_pos_b, ee_quat_b, jacobian_tcp, joint_pos)

    return joint_pos_des

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

# 집게가 목표 열림/닫힘 값에 도착했는지 확인한다.
def is_jaw_reached(jaw_target, is_open_grasp=False):
    jaw_goal = get_jaw_goal(jaw_target, is_open_grasp)
    return torch.all(torch.abs(jaw_goal - jaw_target) < 1.0e-4)

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

# 실제 로봇을 초기 자세로 보내는 명령을 만든다.
def make_real_reset_joint_command():
    return [
        REAL_COMMAND_ZERO_JOINT_POS[joint_name]
        for joint_name in ROBOT_COMMAND_JOINT_NAMES
    ]

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

# 실제 로봇을 초기 자세로 보낸 뒤 잠시 유지한다.
def hold_real_robot_reset(
    sim,
    scene,
    robot,
    robot_entity_cfg,
    jaw_id,
    jaw_target,
    joint_pos_des,
    robot_joint_pub_node,
    sim_dt,
):
    reset_command = make_real_reset_joint_command()
    wait_until = time.monotonic() + INITIAL_REAL_ROBOT_WAIT_SECONDS
    print(
        f"[INFO]: Holding robot reset for {INITIAL_REAL_ROBOT_WAIT_SECONDS:.1f}s..."
    )

    while simulation_app.is_running() and time.monotonic() < wait_until:
        robot_joint_pub_node.set_pose_callback(reset_command)
        robot.set_joint_position_target(
            joint_pos_des,
            joint_ids=robot_entity_cfg.joint_ids,
        )
        robot.set_joint_position_target(jaw_target, joint_ids=[jaw_id])
        scene.write_data_to_sim()
        sim.step()
        scene.update(sim_dt)

        remaining_time = wait_until - time.monotonic()
        if remaining_time > 0.0:
            time.sleep(min(sim_dt, remaining_time))

# 시뮬레이터를 돌리며 큐브를 잡고 옮기는 전체 흐름을 실행한다.
def run_simulator(sim: sim_utils.SimulationContext, scene: InteractiveScene):
    cube_node = GetCubePose()
    robot_joint_pub_node = SetRobotPose()
    robot_joint_sub_node = GetRobotPose()

    robot = scene["robot"]
    cube = scene['cube']

    # ==== IK 설정 ====
    # position, pose에 대해서 ik_command에 실행값을 수정하자
    diff_ik_cfg = DifferentialIKControllerCfg(
        command_type= "position",
        use_relative_mode= False,  # 로봇 베이스 좌표계의 절대 위치
        ik_method= "pinv",
    )
    diff_ik_controller = DifferentialIKController(
        diff_ik_cfg,
        num_envs= scene.num_envs,
        device= sim.device,
    )

    # ==== EE와 목표 Marker 설정 ====
    frame_marker_cfg = FRAME_MARKER_CFG.copy()
    frame_marker_cfg.markers["frame"].scale = (0.1, 0.1, 0.1)
    ee_marker = VisualizationMarkers(frame_marker_cfg.replace(prim_path="/Visuals/ee_current"))
    goal_marker = VisualizationMarkers(frame_marker_cfg.replace(prim_path="/Visuals/ee_goal"))

    # ==== 목표 위치 설정 ====
    # action store에 버퍼 생성
    ik_commands = torch.zeros(scene.num_envs, diff_ik_controller.action_dim, device=robot.device)

    # robot의 joint 설정
    robot_entity_cfg = SceneEntityCfg(
        "robot",
        joint_names=["Rotation", "Pitch", "Elbow", "Wrist_Pitch", "Wrist_Roll"],
        body_names=["gripper"]
    )
    robot_entity_cfg.resolve(scene)
    jaw_id = robot.data.joint_names.index("Jaw")
    jaw_target = torch.full(
        (scene.num_envs, 1),
        OPEN_JAW,
        device=robot.device,
    )

    if robot.is_fixed_base:
        ee_jacobi_idx = robot_entity_cfg.body_ids[0] - 1
    else:
        ee_jacobi_idx = robot_entity_cfg.body_ids[0]

    sim_dt = sim.get_physics_dt() # 0.01초
    count = 0
    ik_move_cnt = 0
    position = None
    # simulation (Pick And Place Pipeline)\
    while simulation_app.is_running():
        if args_cli.sim_to_real:
            if count % 10 == 0:
                # 전송할 회전값 제작
                current_real_joint_command = make_real_joint_command(
                    robot.data.joint_pos[:, robot_entity_cfg.joint_ids],
                    jaw_target,
                )
                # 회전값 전송
                robot_joint_pub_node.set_pose_callback(
                    current_real_joint_command.tolist()
                )
                check_joint_pos = current_real_joint_command.copy().tolist()[:-1]
            # 시뮬과 로봇의 회전 값 간극이 ths이내인지
            if not is_real_robot_synced_with_sim(
                robot_joint_sub_node, # 실제 로봇 회전 값
                # robot.data.joint_pos[:, robot_entity_cfg.joint_ids], # 현재로봇 jaw제외한 회전값
                check_joint_pos, 
                jaw_target, # jaw값
                args_cli.sim_to_real_diff,
            ):
                ik_move_cnt += 1
                if ik_move_cnt > 50:
                    ik_move_cnt = 0
                else:
                    continue

        # 일정 step지나면 리셋
        if count % MAX_STEP == 0:
            reach_count = 0
            command_num = 0
            is_target_position_set = False

            # reset time
            count = 0
            # joint상태 리셋
            joint_pos, joint_vel = make_reset_joint_state(robot)
            robot.write_joint_state_to_sim(joint_pos, joint_vel)
            robot.set_joint_position_target(joint_pos)
            robot.reset()
            jaw_target[:] = OPEN_JAW

            # Basket위치 셋팅
            basket_position = cube_node.get_basket_position()
            if basket_position is None:
                rclpy.spin_once(cube_node)
                continue
            basket_position[2] = 0.18

            # Cube위치 셋팅
            position = cube_node.get_position()
            if position is None:
                rclpy.spin_once(cube_node)
                continue
            
            position[2] = 0.01
            cube_pos_b = torch.tensor(
                position,
                device=scene.device,
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
                    sim_dt=sim_dt,
                )
                check_joint_pos = make_real_reset_joint_command()[:-1]
            # reset controller
            diff_ik_controller.reset()
            # position 모드도 API상 현재 EE 방향을 요구하지만,
            # IK 계산에는 위치 오차와 Jacobian의 위치 부분만 사용한다.
            ee_pose_w = robot.data.body_pose_w[:, robot_entity_cfg.body_ids[0]]
            root_pose_w = robot.data.root_pose_w
            _, ee_quat_b = subtract_frame_transforms(
                root_pose_w[:, 0:3], root_pose_w[:, 3:7], ee_pose_w[:, 0:3], ee_pose_w[:, 3:7]
            )
            # 1. cube world 위치 가져오기
            cube_pos_w = cube.data.root_pos_w
            cube_quat_w = cube.data.root_quat_w

            # 2. robot base 기준 cube 위치로 변환
            cube_pos_b, cube_quat_b = subtract_frame_transforms(
                robot.data.root_pose_w[:, 0:3],
                robot.data.root_pose_w[:, 3:7],
                cube_pos_w,
                cube_quat_w,
            )

            # 3. IK 목표로 설정
            # gripper link가 아니라 URDF의 gripper_tcp가 cube 위로 오도록 한다.
            ik_commands[:] = cube_pos_b
            ik_commands[:, 2] += 0.08

            # 목표위치 저장
            diff_ik_controller.set_command(ik_commands, ee_quat=ee_quat_b)
        else:
            # 물체 위로 다가가기
            if command_num == 0:
                joint_pos_des = move_jacobian(
                    robot= robot,
                    ee_jacobi_idx= ee_jacobi_idx,
                    robot_entity_cfg= robot_entity_cfg,
                    diff_ik_controller= diff_ik_controller,
                )

            # 물체 다가가기 | 물체 내리기
            elif command_num == 1 or command_num == 5:
                if is_target_position_set == False:
                    if command_num == 1:
                        ik_commands[:, 2] -= 0.075
                    elif command_num == 5:
                        ik_commands[:, 0] = basket_position[0]
                        ik_commands[:, 1] = basket_position[1]
                        ik_commands[:, 2] = basket_position[2]
                    diff_ik_controller.set_command(ik_commands, ee_quat=ee_quat_b)
                    is_target_position_set = True
                joint_pos_des = move_jacobian(
                    robot= robot,
                    ee_jacobi_idx= ee_jacobi_idx,
                    robot_entity_cfg= robot_entity_cfg,
                    diff_ik_controller= diff_ik_controller,
                )

            # 물체 잡기
            elif command_num == 2:
                jaw_target = move_grasp(
                    jaw_target= jaw_target,
                    sim_dt= sim_dt,
                    is_open_grasp=False
                )

            # 물체 들기
            elif command_num == 3:
                if is_target_position_set == False:
                    ik_commands[:, 2] = basket_position[2]
                    diff_ik_controller.set_command(ik_commands, ee_quat=ee_quat_b)
                    is_target_position_set = True
                joint_pos_des = move_jacobian(
                    robot= robot,
                    ee_jacobi_idx= ee_jacobi_idx,
                    robot_entity_cfg= robot_entity_cfg,
                    diff_ik_controller= diff_ik_controller,
                )

            # 물체 이동하기
            elif command_num == 4:
                if is_target_position_set == False:
                    ik_commands[:, 0] = basket_position[0]
                    ik_commands[:, 1] = basket_position[1]
                    ik_commands[:, 2] = basket_position[2]
                    # ik_commands[:, 0] += 0.06
                    # ik_commands[:, 1] += 0.06
                    diff_ik_controller.set_command(ik_commands, ee_quat=ee_quat_b)
                    is_target_position_set = True
                joint_pos_des = move_jacobian(
                    robot= robot,
                    ee_jacobi_idx= ee_jacobi_idx,
                    robot_entity_cfg= robot_entity_cfg,
                    diff_ik_controller= diff_ik_controller,
                )

            # 물체 내리기

            # 물체 놓기
            elif command_num == 6:
                jaw_target = move_grasp(
                    jaw_target= jaw_target,
                    sim_dt= sim_dt,
                    is_open_grasp=True
                )

            # # 다음 step으로 갈지 검증
            dist = torch.norm(tcp_pos_w - goal_pos_w, dim=1)

            if command_num == 2:
                is_command_done = is_jaw_reached(jaw_target, is_open_grasp=False)
            elif command_num == 6:
                is_command_done = is_jaw_reached(jaw_target, is_open_grasp=True)
            else:
                is_command_done = torch.all(dist < COMMAND_THRESHOLD[command_num])

            if command_num == 0:
                is_command_done = torch.all(dist < COMMAND_THRESHOLD[command_num])

            if is_command_done:
                reach_count += 1
            else:
                reach_count = 0

            if reach_count > 20:
                command_num += 1
                is_target_position_set = False
                reach_count = 0
            # print(f"{command_num}: {dist}")
            if command_num == 7:
                count = MAX_STEP - 1

        # 행동 실행
        # 팔관절
        robot.set_joint_position_target(joint_pos_des, joint_ids=robot_entity_cfg.joint_ids)
        # gripper 관절
        robot.set_joint_position_target(jaw_target, joint_ids=[jaw_id])
        scene.write_data_to_sim()
        # perform step
        sim.step()
        # update sim-time
        count += 1
        # update buffers
        scene.update(sim_dt)

        # obtain quantities from simulation
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
        # update marker positions
        ee_marker.visualize(tcp_pos_w, tcp_quat_w)
        goal_marker.visualize(goal_pos_w, goal_quat_w)

# ROS와 Isaac Sim을 준비하고 시뮬레이션을 시작한다.
def main():
    try:
        rclpy.init()
        # Load kit helper
        sim_cfg = sim_utils.SimulationCfg(dt=0.01, device=args_cli.device)
        sim = sim_utils.SimulationContext(sim_cfg)
        # Set main camera
        sim.set_camera_view([2.5, 2.5, 2.5], [0.0, 0.0, 0.0])
        # Design scene
        scene_cfg = TableTopSceneCfg(num_envs=args_cli.num_envs, env_spacing=2.0)
        scene = InteractiveScene(scene_cfg)
        # Play the simulator
        sim.reset()
        # Now we are ready!
        print("[INFO]: Setup complete...")
        # Run the simulator
        run_simulator(sim, scene)
    finally:
        rclpy.shutdown()

if __name__ == "__main__":
    # run the main function
    main()
    # close sim app
    simulation_app.close()