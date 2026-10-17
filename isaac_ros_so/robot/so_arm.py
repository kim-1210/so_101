from pathlib import Path

import isaaclab.sim as sim_utils
from isaaclab.actuators import ImplicitActuatorCfg
from isaaclab.assets.articulation import ArticulationCfg

So_Arm_CFG = ArticulationCfg(
    spawn=sim_utils.UsdFileCfg(
        usd_path="/media/kim/hdd1tb2/master_study/so_101/isaac_ros_so/robot/so_arm.usd",
        activate_contact_sensors=False,

        rigid_props=sim_utils.RigidBodyPropertiesCfg(
            disable_gravity=False,
            max_depenetration_velocity=1.0,
        ),
        articulation_props=sim_utils.ArticulationRootPropertiesCfg(
            enabled_self_collisions=True,
            solver_position_iteration_count=32,
            solver_velocity_iteration_count=8
        ),
    ),

    init_state=ArticulationCfg.InitialStateCfg(
        joint_pos={
            "Rotation": 0.0084,
            "Pitch": -0.602,
            "Elbow": -0.2631,
            "Wrist_Pitch": 0.639,
            "Wrist_Roll": -2.7,
            "Jaw": 0.1081,
        },
    ),

    actuators={
        "arm": ImplicitActuatorCfg(
            joint_names_expr=[
                "Rotation", 
                "Pitch", 
                "Elbow",
                "Wrist_Pitch",
                "Wrist_Roll",
            ],
            effort_limit_sim=3.35,
            velocity_limit_sim=4.72,
            stiffness=17.8,
            damping=0.60,
        ),
        "gripper": ImplicitActuatorCfg(
            joint_names_expr=["Jaw",],
            effort_limit_sim=3.35,
            velocity_limit_sim=4.72,
            stiffness=17.8,
            damping=0.60,
        ),
    }
)
