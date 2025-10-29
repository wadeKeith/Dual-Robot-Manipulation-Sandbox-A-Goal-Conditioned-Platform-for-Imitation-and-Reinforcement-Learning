import math
import pickle
import random
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
import torch
from tqdm import tqdm

import rl_utils
from DDPG_her import DDPG, PolicyNet, ReplayBuffer_Trajectory, Trajectory
from pick_place_env import PickPlaceUR5eEnv

def evluation_policy(env, state_dim, action_dim, hidden_dim, device, model_num, model_dir: Path):
    model = PolicyNet(state_dim, hidden_dim, action_dim).to(device)
    model.load_state_dict(torch.load(model_dir / f"ddpg_her_ur5_pick_{model_num}.pkl"))
    model.eval()
    episode_return = 0
    state,_,_ = env.reset()
    done = False
    while not done:
        state = torch.tensor(state, dtype=torch.float).to(device)
        action = model(state).detach().cpu().numpy()
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
sim_params = {"use_gui":False,
              'timestep':1/240,
              'control_type':'end',
              'gripper_enable':True,
              'is_train':True,
              'distance_threshold':0.05,}
# env_kwargs_dict = {"sim_params":sim_params, "robot_params": robot_params, "visual_sensor_params": visual_sensor_params}

use_expert_data = True
model_dir = Path("./model")
model_dir.mkdir(parents=True, exist_ok=True)
buffer_snapshot_path = Path("ddpg_her_buffer_pickplace_all.pkl")
expert_data_path = Path("ur5_pickplace_10000_expert_data.pkl")

env = PickPlaceUR5eEnv(sim_params, robot_params, visual_sensor_params)

state_dim = env.observation_space['observation'].shape[0]+env.observation_space['desired_goal'].shape[0]+env.observation_space['achieved_goal'].shape[0]
action_dim = env.action_space.shape[0]



actor_lr = 1e-3
critic_lr = 1e-3
num_episodes = 100
hidden_dim = 128
gamma = 0.99999
sigma = 0.5
tau = 0.01  # 软更新参数
buffer_size = 100000
minimal_episodes = 5
n_train = 5
batch_size = 512
state_len = env.observation_space['observation'].shape[0]
achieved_goal_len = env.observation_space['achieved_goal'].shape[0]
if torch.cuda.is_available():
    device = torch.device("cuda")
elif hasattr(torch.backends, "mps") and torch.backends.mps.is_available():
    device = torch.device("mps")
else:
    device = torch.device("cpu")




if use_expert_data:
    with expert_data_path.open("rb") as f:
        her_buffer = pickle.load(f)
else:
    her_buffer = ReplayBuffer_Trajectory(capacity= buffer_size, 
                                        dis_threshold=sim_params['distance_threshold'], 
                                        use_her=True,
                                        batch_size=batch_size,
                                        state_len=state_len,
                                        achieved_goal_len=achieved_goal_len,)
    
agent = DDPG(state_dim, hidden_dim, action_dim,
                 actor_lr, critic_lr, sigma, tau, gamma, device)

load_agent = False 
agent_num = 52
if load_agent:
    agent.actor.load_state_dict(torch.load(model_dir / f"ddpg_her_ur5_pick_{agent_num}.pkl"))

return_list = []
for i in range(100):
    agent.lr_decay(i)
    with tqdm(total=int(num_episodes), desc='Iteration %d' % i) as pbar:
        success_count = 0
        for i_episode in range(num_episodes):
            episode_return = 0
            state,_,_ = env.reset()
            traj = Trajectory(state.copy())
            done = False
            while not done:
                with torch.no_grad():
                    action = agent.take_action(state)
                state, reward, terminated, truncated, info,_ = env.step(action)
                done = terminated or truncated
                episode_return += reward
                traj.store_step(action.copy(), state.copy(), reward, done)
            her_buffer.add_trajectory(traj)
            return_list.append(episode_return)
            if info['is_success'] == True:
                success_count+=1
                # her_buffer.add_trajectory(traj)
            if her_buffer.size() >= minimal_episodes:
                # her_buffer_len_ls = her_buffer.buffer[-1].length
                # her_buffer_minlen_ls = [her_buffer.buffer[i].length for i in range(her_buffer.size())]
                # her_ratio = (her_buffer_len_ls-1)/env.time_limitation
                her_ratio = 1
                for _ in range(n_train):
                    transition_dict = her_buffer.sample(her_ratio)
                    agent.update(transition_dict)
                pbar.set_postfix({
                    'goal':
                    '%r' % (env.goal),
                    # 'her_bf_min_len': min(her_buffer_minlen_ls),
                    'her_size':her_buffer.size(),
                    'episode':
                        '%d' % (num_episodes* i + i_episode + 1),
                    "her dones":np.count_nonzero(transition_dict['dones']),
                    'return':
                    '%.3f' % np.mean(return_list[:]),
                    "lr": agent.actor_optimizer.param_groups[0][
                                "lr"],
                    "is success count": success_count,
                    # "HER ratio":her_ratio
                })
            else:
                pbar.set_postfix({
                    'goal':
                    '%r' % (env.goal),
                    'episode':
                        '%d' % (num_episodes* i + i_episode + 1),
                    'return':
                    '%.3f' % np.mean(return_list[:]),
                    "lr": agent.actor_optimizer.param_groups[0][
                                "lr"],
                    "is success count": success_count,
                })
            pbar.update(1)
    torch.save(agent.actor.state_dict(), model_dir / f"ddpg_her_ur5_pick_{i}.pkl")
    sim_params['is_train'] = False
    # sim_params['use_gui'] = True
    test_env  = PickPlaceUR5eEnv(sim_params, robot_params,visual_sensor_params)
    evluation_policy(
        env=test_env,
        state_dim=agent.state_dim,
        action_dim=agent.action_dim,
        hidden_dim=agent.hidden_dim,
        device=agent.device,
        model_num=i,
        model_dir=model_dir,
    )
    test_env.close()
    del test_env
    sim_params['is_train'] = True
    # sim_params['use_gui'] = False

env.close()
del env
with buffer_snapshot_path.open('wb') as file:
    pickle.dump(her_buffer, file)
episodes_list = list(range(len(return_list)))
plt.plot(episodes_list, return_list)
plt.xlabel('Episodes')
plt.ylabel('Returns')
plt.title('DDPG with HER on {}'.format('UR5'))
plt.show()

mv_return = rl_utils.moving_average(return_list, 9)
plt.plot(episodes_list, mv_return)
plt.xlabel('Episodes')
plt.ylabel('Returns')
plt.title('DDPG on {}'.format('UR5'))
plt.show()
