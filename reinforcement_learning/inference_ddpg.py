import math
import random
from pathlib import Path

import numpy as np
import torch

from DDPG_her import PolicyNet
from pick_place_env import PickPlaceUR5eEnv

def evluation_policy(env, state_dim, action_dim, hidden_dim, device, model_path: Path):
    model = PolicyNet(state_dim, hidden_dim, action_dim).to(device)
    model.load_state_dict(torch.load(model_path))
    model.eval()
    episode_return = 0
    state,_,_ = env.reset()
    # env.goal = env.handle_pos+np([0.1,0.1,0.05])
    done = False
    while not done:
        state = torch.tensor(state, dtype=torch.float).to(device)
        action = model(state).detach().cpu().numpy()
        state, reward, terminated, truncated, info,_ = env.step(action)
        done = terminated or truncated
        episode_return += reward
    print(
        "Test reward %.3f and info: is_success: %r, goal is %r"
        % (episode_return, info["is_success"], env.goal)
    )



seed = 3407
random.seed(seed)
np.random.seed(seed)
torch.manual_seed(seed)
torch.cuda.manual_seed(seed)
torch.cuda.manual_seed_all(seed)
reset_arm_poses = [0, -math.pi/2, math.pi*4/9, -math.pi*4/9,
                            -math.pi/2, 0]
reset_gripper_range = [0, 0.085]
visual_sensor_params = {
        'image_size': [128, 128],
        'dist': 1.0,
        'yaw': 90.0,
        'pitch': -25.0,
        'pos': [0.6, 0.0, 0.0525],
        'fov': 75.0,
        'near_val': 0.1,
        'far_val': 5.0,
        'show_vision': False
    }
robot_params = {
    "reset_arm_poses": reset_arm_poses,
    "reset_gripper_range": reset_gripper_range,
}
# control type: joint, end
sim_params = {"use_gui":True,
              'timestep':1/240,
              'control_type':'end',
              'gripper_enable':True,
              'is_train':False,
              'distance_threshold':0.05,}
# env_kwargs_dict = {"sim_params":sim_params, "robot_params": robot_params, "visual_sensor_params": visual_sensor_params}

model_dir = Path("./model")
model_path = model_dir / "ddpg_her_ur5_pick_52.pkl"

env = PickPlaceUR5eEnv(sim_params, robot_params, visual_sensor_params)

if torch.cuda.is_available():
    device = torch.device("cuda")
elif hasattr(torch.backends, "mps") and torch.backends.mps.is_available():
    device = torch.device("mps")
else:
    device = torch.device("cpu")




hidden_dim = 128
    

state_dim = (
    env.observation_space["observation"].shape[0]
    + env.observation_space["desired_goal"].shape[0]
    + env.observation_space["achieved_goal"].shape[0]
)
action_dim = env.action_space.shape[0]

evluation_policy(
    env=env,
    state_dim=state_dim,
    action_dim=action_dim,
    hidden_dim=hidden_dim,
    device=device,
    model_path=model_path,
)
env.close()
del env
