import warnings
warnings.filterwarnings(action='ignore')

import argparse
from isaaclab.app import AppLauncher

def parse_boolean(value):
    normalized_value = value.lower()
    if normalized_value in ("true", "1", "yes", "y", "on"):
        return True
    if normalized_value in ("false", "0", "no", "n", "off"):
        return False
    raise argparse.ArgumentTypeError("true 또는 false를 입력해주세요.")

parser = argparse.ArgumentParser(description="So arm diff IK")
parser.add_argument("--robot", type=str, default="so_arm")
parser.add_argument("--num_envs", type=int, default=10)
parser.add_argument("--sim_to_real", type=parse_boolean, default=False)
parser.add_argument(
    "--sim_to_real_diff",
    type=float,
    default=0.015,
    help="실제 관절이 명령에 도착했다고 볼 허용 오차(rad, 최대 0.02)",
)
# parser.add_argument("--x", type=float, default=0.55)
# parser.add_argument("--y", type=float, default=0.0)
# parser.add_argument("--z", type=float, default=1.08)

# isaac sim을 실행하는데 부족한 args를 채워줌
AppLauncher.add_app_launcher_args(parser)

args_cli = parser.parse_args()

app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

import torch
import numpy as np

import isaaclab.sim as sim_utils
from isaaclab.assets import AssetBaseCfg, RigidObjectCfg, ArticulationCfg
from isaaclab.controllers import DifferentialIKController, DifferentialIKControllerCfg
from isaaclab.managers import SceneEntityCfg
from isaaclab.markers import VisualizationMarkers
from isaaclab.markers.config import FRAME_MARKER_CFG
from isaaclab.scene import InteractiveScene, InteractiveSceneCfg
from isaaclab.sim.spawners.materials.physics_materials_cfg import RigidBodyMaterialCfg
from isaaclab.utils import configclass
from isaaclab.utils.assets import ISAAC_NUCLEUS_DIR
from isaaclab.utils.math import subtract_frame_transforms
from isaaclab.utils.math import quat_apply, skew_symmetric_matrix
from isaaclab.sim.spawners.from_files.from_files_cfg import GroundPlaneCfg, UsdFileCfg
from isaaclab.utils.math import subtract_frame_transforms, combine_frame_transforms

from robot.so_arm import So_Arm_CFG
