import numpy as np
from utilize import connect_pybullet, set_debug_camera, Camera, distance
import random
import numpy as np
from tqdm import tqdm
import torch
import matplotlib.pyplot as plt
import math
import pickle
import time


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
              'is_train':True,
              'distance_threshold':0.05,}

vis = sim_params['use_gui']
pb = connect_pybullet(sim_params['timestep'], show_gui=vis)
base_pos = [0, 0, 0]
base_rpy = [0, 0, 0]
base_orn = pb.getQuaternionFromEuler(base_rpy)
asset_name = "./assets/urdf/dual_robot.urdf"
dual_robot = pb.loadURDF(asset_name, base_pos, base_orn)


while True:
    pb.stepSimulation()
    time.sleep(sim_params['timestep'])