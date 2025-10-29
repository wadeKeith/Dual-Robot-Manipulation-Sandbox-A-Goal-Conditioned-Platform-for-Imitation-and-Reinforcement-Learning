import sys
import os
current_directory = os.getcwd()
sys.path.append(current_directory)
import numpy as np
from pick_place_env import PickPlace_UR5Env
import random
import numpy as np
import torch
import matplotlib.pyplot as plt
from WGCSL import PolicyNet
import math

def evluation_policy(env, state_dim, action_dim,hidden_dim, device, model_num):
    model = PolicyNet(state_dim, hidden_dim, action_dim).to(device)
    model.load_state_dict(torch.load(os.path.join(current_directory,"model/wgcsl_her_dual_robot_pick_actor_%d.pkl" % model_num)))
    model.eval()
    episode_return = 0
    state,_,_ = env.reset()
    done = False
    while not done:
        state = torch.tensor(state, dtype=torch.float).to(device)
        mu,_ = model(state)
        action = mu.detach().cpu().numpy()
        state, reward, terminated, truncated, info,_ = env.step(action)
        done = terminated or truncated
        episode_return += reward
    print("Test rawrd of the model %d is %.3f and info: is_success: %r, goal is %r" % (model_num, episode_return, info['is_success'],env.goal))



seed = 3407
random.seed(seed)
np.random.seed(seed)
torch.manual_seed(seed)
torch.cuda.manual_seed(seed)
torch.cuda.manual_seed_all(seed)
reset_arm_poses = [0, -math.pi/2, -math.pi*5/9, math.pi*4/9, math.pi*3/4, 0, 
                   0, math.pi/2, math.pi*5/9, -math.pi*4/9, -math.pi*3/4, 0]
reset_gripper_range = [0, 0.085]
visual_sensor_params = {
        'image_size': [128, 128],
        'dist': 1.5,
        'yaw': 90.0,
        'pitch': -25.0,
        'pos': [0.5, 0.0, 1],
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
              'is_train':False,
              'distance_threshold':0.05,}



env = PickPlace_UR5Env(sim_params, robot_params,visual_sensor_params)

state_dim = env.observation_space['observation'].shape[0]+env.observation_space['desired_goal'].shape[0]+env.observation_space['achieved_goal'].shape[0]
action_dim = env.action_space.shape[0]




state_len = env.observation_space['observation'].shape[0]
achieved_goal_len = env.observation_space['achieved_goal'].shape[0]
device = torch.device("cuda") if torch.cuda.is_available() else torch.device(
    "mps")




hidden_dim = 256
    

evluation_policy(env=env, state_dim=state_dim,
                    action_dim = action_dim,
                    hidden_dim=hidden_dim, 
                    device=device,
                    model_num=25)
env.close()
del env

