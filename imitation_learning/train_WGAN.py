import sys
import os
current_directory = os.getcwd()
sys.path.append(current_directory)
import numpy as np
from pick_place_env import PickPlace_UR5Env
import random
import numpy as np
from tqdm import tqdm
import torch
import matplotlib.pyplot as plt
from WGAN import PPOContinuous, WGAN, PolicyNet,ReplayBuffer_Trajectory
import math
import pickle
import rl_utils

def evluation_policy(env, state_dim, action_dim,hidden_dim, device, model_num):
    model = PolicyNet(state_dim, hidden_dim, action_dim).to(device)
    model.load_state_dict(torch.load(os.path.join(current_directory,"model/wgan_dual_robot_pick_actor_%d.pkl" % model_num)))
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
sim_params = {"use_gui":False,
              'timestep':1/240,
              'control_type':'end',
              'is_train':True,
              'distance_threshold':0.05,}


env = PickPlace_UR5Env(sim_params, robot_params,visual_sensor_params)

state_dim = env.observation_space['observation'].shape[0]+env.observation_space['desired_goal'].shape[0]+env.observation_space['achieved_goal'].shape[0]
action_dim = env.action_space.shape[0]



actor_lr = 3e-5
critic_lr = 3e-5
num_episodes = 100
hidden_dim = 256
gamma = 0.99999
lmbda = 0.95
entropy_coef = 0.01
epochs = 100
eps = 0.15
lr_d = 3e-4
c_lambda = 10
state_len = env.observation_space['observation'].shape[0]
achieved_goal_len = env.observation_space['achieved_goal'].shape[0]
device = torch.device("cuda") if torch.cuda.is_available() else torch.device(
    "mps")


    
agent = PPOContinuous(state_dim, hidden_dim, action_dim, actor_lr, critic_lr, lmbda, epochs, eps, gamma, device, entropy_coef)
wgan = WGAN(agent=agent,state_dim=state_dim,action_dim=action_dim,hidden_dim=hidden_dim,
            lr_d=lr_d,device=device,epochs=epochs,c_lambda = c_lambda)

buffer_size = 100000
batch_size = 512

her_buffer = ReplayBuffer_Trajectory(capacity= buffer_size, 
                                    dis_threshold=sim_params['distance_threshold'], 
                                    use_her=False,
                                    batch_size=batch_size,
                                    state_len=state_len,
                                    achieved_goal_len=achieved_goal_len,)
with open(os.path.join(current_directory,'dual_robot_pickplace_40000_expert_data_WGCSL.pkl'), 'rb') as f:
# 读取并反序列化数据
    her_buffer_buffer = pickle.load(f)
f.close()
her_buffer.buffer = her_buffer_buffer.buffer

return_list = []
transition_dict = {
                    "states": [],
                    "actions": [],
                    "next_states": [],
                    "rewards": [],
                    "dones": [],
                }
for i in range(100):
    agent.lr_decay(i)
    wgan.lr_decay(i)
    with tqdm(total=int(num_episodes), desc='Iteration %d' % i) as pbar:
        success_count = 0
        for i_episode in range(num_episodes):
            episode_return = 0
            state,_,_ = env.reset()
            done = False
            while not done:
                transition_dict['states'].append(state.copy())
                with torch.no_grad():
                    action = agent.take_action(state)
                transition_dict['actions'].append(action.copy())
                state, reward, terminated, truncated, info,_ = env.step(action)
                done = terminated or truncated
                episode_return += reward
                transition_dict['next_states'].append(state.copy())
                transition_dict['rewards'].append(reward)
                transition_dict['dones'].append(done)
            # her_buffer.add_trajectory(traj)
            return_list.append(episode_return)
            if info['is_success'] == True:
                success_count+=1
            # transition_dict,her_info = her_process(transition_dict, state_len, achieved_goal_len,sim_params['distance_threshold'])
            trans_len = len(transition_dict['rewards'])
            her_buffer.batch_size = trans_len
            expert_dict = her_buffer.sample(0)
            discriminator_loss, ppo_reward =wgan.learn(expert_dict['states'], expert_dict['actions'], np.array(transition_dict['states']), np.array(transition_dict['actions']), np.array(transition_dict['next_states']), np.array(transition_dict['dones']))
            transition_dict = {
                    "states": [],
                    "actions": [],
                    "next_states": [],
                    "rewards": [],
                    "dones": [],
                }
            pbar.set_postfix({
                # 'goal':
                # '%r' % (env.goal),
                'ppo_reward':ppo_reward,
                'discriminator_loss':discriminator_loss,
                'episode':
                    '%d' % (num_episodes* i + i_episode + 1),
                'return':
                '%.3f' % np.mean(return_list[:]),
                "lr_a": agent.actor_optimizer.param_groups[0][
                            "lr"],
                "lr_d": wgan.discriminator_optimizer.param_groups[0][
                            "lr"],
                "is success count": success_count,
                # "HER ratio":her_ratio
            })

            pbar.update(1)
    torch.save(agent.actor.state_dict(), os.path.join(current_directory,"model/wgan_dual_robot_pick_actor_%d.pkl" % i))
    torch.save(agent.critic.state_dict(), os.path.join(current_directory,"model/wgan_dual_robot_pick_critic_%d.pkl" % i))
    sim_params['is_train'] = False
    # sim_params['use_gui'] = True
    test_env  = PickPlace_UR5Env(sim_params, robot_params,visual_sensor_params)
    evluation_policy(env=test_env, state_dim=agent.state_dim,
                     action_dim = agent.action_dim,
                     hidden_dim=agent.hidden_dim, 
                     device=agent.device,
                     model_num=i)
    test_env.close()
    del test_env
    sim_params['is_train'] = True
    # sim_params['use_gui'] = False

env.close()
del env

episodes_list = list(range(len(return_list)))
plt.plot(episodes_list, return_list)
plt.xlabel('Episodes')
plt.ylabel('Returns')
plt.title('PPO on {}'.format('UR5'))
plt.show()

mv_return = rl_utils.moving_average(return_list, 9)
plt.plot(episodes_list, mv_return)
plt.xlabel('Episodes')
plt.ylabel('Returns')
plt.title('PPO on {}'.format('UR5'))
plt.show()