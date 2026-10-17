"""
isaaclab_4 -p ./isaac_ros_so/isaac_jacobian_2/main.py \
  --kit_args="--/log/level=error --/log/fileLogLevel=error --/log/outputStreamLevel=error" \
  --num_envs 1 \
  --sim_to_real True \
  --sim_to_real_diff 0.015

ros2 topic pub --once /follower/joint_commands std_msgs/msg/Float64MultiArray "{data: [-1.0, -1.0, 0.0, 0.0, 0.0, 0.0]}"
"""

from etc import *

def run_simulator(sim: sim_utils.SimulationContext, scene: InteractiveScene):
    cube_node = GetCubePose()
    robot_joint_pub_node = SetRobotPose()
    robot_joint_sub_node = GetRobotPose()

    robot = scene["robot"]
    cube = scene['cube']

    diff_ik_controller = load_IK(sim, scene)

    ee_marker, goal_marker = setup_markers()
    ik_commands = torch.zeros(scene.num_envs, diff_ik_controller.action_dim, device=robot.device)

    robot_entity_cfg = set_robot_joint_cfg()
    robot_entity_cfg.resolve(scene)
    ee_jacobi_idx = get_ee_jacobi_idx(robot, robot_entity_cfg)

    jaw_id = robot.data.joint_names.index("Jaw")
    jaw_target = torch.full(
        (scene.num_envs, 1),
        OPEN_JAW,
        device=robot.device,
    )

    sim_dt = sim.get_physics_dt() # 0.01초
    state = {
        "count": 0,
        "needs_reset": True,
        "sync_state": make_real_sync_state(),
    }

    while simulation_app.is_running() and rclpy.ok():
        # count=0에서는 Isaac의 초기 자세를 실제 로봇에 먼저 보내지 않는다.
        # reset 자세와 작업 목표를 준비한 뒤에만 실로봇 동기화를 시작한다.
        if state["needs_reset"]:
            reset_state = reset_task(
                sim=sim,
                scene=scene,
                robot=robot,
                cube=cube,
                cube_node=cube_node,
                robot_joint_pub_node=robot_joint_pub_node,
                robot_joint_sub_node=robot_joint_sub_node,
                robot_entity_cfg=robot_entity_cfg,
                jaw_id=jaw_id,
                jaw_target=jaw_target,
                ik_commands=ik_commands,
                diff_ik_controller=diff_ik_controller,
                sim_dt=sim_dt,
            )
            if reset_state is None:
                continue
            state.update(reset_state)
            reset_real_sync_state(state["sync_state"])
        else:
            can_move = sync_real_robot(
                robot=robot,
                robot_entity_cfg=robot_entity_cfg,
                jaw_target=jaw_target,
                robot_joint_pub_node=robot_joint_pub_node,
                robot_joint_sub_node=robot_joint_sub_node,
                count=state["count"],
                sync_state=state["sync_state"],
            )
            if not can_move:
                continue

            jaw_target = run_pick_place_step(
                state=state,
                robot=robot,
                robot_entity_cfg=robot_entity_cfg,
                ee_jacobi_idx=ee_jacobi_idx,
                diff_ik_controller=diff_ik_controller,
                ik_commands=ik_commands,
                jaw_target=jaw_target,
                sim_dt=sim_dt,
            )

        step_sim(sim, scene, robot, robot_entity_cfg, jaw_id, state, jaw_target, sim_dt)
        state["tcp_pos_w"], state["goal_pos_w"] = update_markers(
            scene=scene,
            robot=robot,
            robot_entity_cfg=robot_entity_cfg,
            ik_commands=ik_commands,
            ee_marker=ee_marker,
            goal_marker=goal_marker,
        )

def main():
    node_started = False
    try:
        rclpy.init()
        node_started = True

        sim_cfg = sim_utils.SimulationCfg(dt=0.01, device=args_cli.device)
        sim = sim_utils.SimulationContext(sim_cfg)
        sim.set_camera_view([2.5, 2.5, 2.5], [0.0, 0.0, 0.0])

        scene_cfg = TableTopSceneCfg(num_envs=args_cli.num_envs, env_spacing=2.0)
        scene = InteractiveScene(scene_cfg)

        sim.reset()

        print("[INFO]: Setup complete...")
        run_simulator(sim, scene)
    finally:
        if node_started and rclpy.ok():
            rclpy.shutdown()
        simulation_app.close()

if __name__ == "__main__":
    main()
