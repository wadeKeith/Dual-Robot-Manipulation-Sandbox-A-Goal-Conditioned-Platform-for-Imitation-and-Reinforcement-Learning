import numpy as np
from utilize import connect_pybullet, set_debug_camera, Camera, distance
import random
from pick_place_env import PickPlace_UR5Env
import numpy as np
from tqdm import tqdm
import torch
import matplotlib.pyplot as plt
import math
import pickle
import time

seed = 3407
random.seed(seed)
np.random.seed(seed)
torch.manual_seed(seed)
torch.cuda.manual_seed(seed)
torch.cuda.manual_seed_all(seed)
reset_arm_poses = [0, -math.pi/2, -math.pi*5/9, math.pi*4/9, math.pi*3/4, 0, 
                   0, math.pi/2, math.pi*5/9, -math.pi*4/9, -math.pi*3/4, 0]
# reset_arm_poses = [0, 0, 0, 0, 0, 0, 
#                     0, 0, 0, 0, 0, 0]
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
              'gripper_enable':True,
              'is_train':True,
              'distance_threshold':0.05,}

env =  PickPlace_UR5Env(sim_params=sim_params,
                        robot_params= robot_params,
                        visual_sensor_params= visual_sensor_params)

obs, _,obs_dict = env.reset()
# obs, reward, terminated, truncated, info,obs_dict = env.step(np.array([0,0,0,0,0,0,0,0]))
while True:
    # obs, _,obs_dict = env.reset()
    obs, reward, terminated, truncated, info,obs_dict = env.step(np.array(env.read_debug_parameter()))
    # env.step_simulation()
    # time.sleep(sim_params['timestep'])