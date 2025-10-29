import argparse
import math
import random

import numpy as np
import torch

from pick_place_env import PickPlaceUR5eEnv, PickPlace_UR5Env
from reach_env import ReachUR5Env


def set_seed(seed: int) -> None:
    random.seed(seed)
    np.random.seed(seed)
    torch.manual_seed(seed)
    if torch.cuda.is_available():
        torch.cuda.manual_seed(seed)
        torch.cuda.manual_seed_all(seed)


def build_dual_env(use_gui: bool, is_train: bool):
    reset_arm_poses = [
        0,
        -math.pi / 2,
        -math.pi * 5 / 9,
        math.pi * 4 / 9,
        math.pi * 3 / 4,
        0,
        0,
        math.pi / 2,
        math.pi * 5 / 9,
        -math.pi * 4 / 9,
        -math.pi * 3 / 4,
        0,
    ]
    robot_params = {
        "reset_arm_poses": reset_arm_poses,
        "reset_gripper_range": [0, 0.085],
    }
    sim_params = {
        "use_gui": use_gui,
        "timestep": 1 / 240,
        "control_type": "end",
        "is_train": is_train,
        "distance_threshold": 0.05,
    }
    visual_sensor_params = {
        "image_size": [128, 128],
        "dist": 1.5,
        "yaw": 90.0,
        "pitch": -25.0,
        "pos": [0.5, 0.0, 1.0],
        "fov": 75.0,
        "near_val": 0.1,
        "far_val": 5.0,
        "show_vision": False,
    }
    env = PickPlace_UR5Env(sim_params, robot_params, visual_sensor_params)
    return env, "dual pick-and-place"


def build_single_env(use_gui: bool, is_train: bool):
    reset_arm_poses = [0, -math.pi / 2, math.pi * 4 / 9, -math.pi * 4 / 9, -math.pi / 2, 0]
    robot_params = {
        "reset_arm_poses": reset_arm_poses,
        "reset_gripper_range": [0, 0.085],
    }
    sim_params = {
        "use_gui": use_gui,
        "timestep": 1 / 240,
        "control_type": "end",
        "gripper_enable": True,
        "is_train": is_train,
        "distance_threshold": 0.05,
    }
    visual_sensor_params = {
        "image_size": [128, 128],
        "dist": 1.0,
        "yaw": 90.0,
        "pitch": -25.0,
        "pos": [0.6, 0.0, 0.0525],
        "fov": 75.0,
        "near_val": 0.1,
        "far_val": 5.0,
        "show_vision": False,
    }
    env = PickPlaceUR5eEnv(sim_params, robot_params, visual_sensor_params)
    return env, "single-arm pick-and-place"


def build_reach_env(use_gui: bool, is_train: bool):
    reset_arm_poses = [0, -math.pi / 2, math.pi * 4 / 9, -math.pi * 4 / 9, -math.pi / 2, 0]
    robot_params = {
        "reset_arm_poses": reset_arm_poses,
        "reset_gripper_range": [0, 0.085],
    }
    sim_params = {
        "use_gui": use_gui,
        "timestep": 1 / 240,
        "control_type": "end",
        "gripper_enable": False,
        "is_train": is_train,
        "distance_threshold": 0.05,
    }
    visual_sensor_params = {
        "image_size": [128, 128],
        "dist": 1.0,
        "yaw": 90.0,
        "pitch": -25.0,
        "pos": [0.6, 0.0, 0.0525],
        "fov": 75.0,
        "near_val": 0.1,
        "far_val": 5.0,
        "show_vision": False,
    }
    env = ReachUR5Env(sim_params, robot_params, visual_sensor_params)
    return env, "single-arm reaching"


def main():
    parser = argparse.ArgumentParser(description="Quick environment smoke test")
    parser.add_argument("--robot-type", choices=["dual", "single", "reach"], default="dual")
    parser.add_argument("--no-gui", action="store_true", help="Run without PyBullet GUI")
    parser.add_argument("--train-mode", action="store_true", help="Sample random initial/goal configurations")
    parser.add_argument("--seed", type=int, default=3407)
    args = parser.parse_args()

    set_seed(args.seed)
    builders = {
        "dual": build_dual_env,
        "single": build_single_env,
        "reach": build_reach_env,
    }
    env, description = builders[args.robot_type](use_gui=not args.no_gui, is_train=args.train_mode)
    print(f"Loaded {description} environment; action dim {env.action_space.shape[0]}")

    obs, *_ = env.reset()
    zero_action = np.zeros(env.action_space.shape, dtype=np.float32)

    for step in range(120):
        obs, reward, terminated, truncated, info, *rest = env.step(zero_action)
        if terminated or truncated:
            print(f"Episode ended at step {step}: info={info}")
            env.reset()

    env.close()
if __name__ == "__main__":
    main()
