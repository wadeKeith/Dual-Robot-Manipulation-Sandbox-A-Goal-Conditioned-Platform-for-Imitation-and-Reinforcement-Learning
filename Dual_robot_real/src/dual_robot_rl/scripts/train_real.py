import numpy as np
import rospy
from std_msgs.msg import String
import sys
sys.path.insert(0, "/media/jt/Extreme SSD/Github/Dual_robot_real/src/dual_robot_rl/scripts")
from utils.common import  log_ros_params, clear_gym_params, load_ros_params, ROSLauncher
from ur5_task_space_pick_and_place_real import UR5PickAndPlaceEnv
import torch
from rl_method.ppo import PPOContinuous, PolicyNet, her_process
from rl_method import rl_utils
import matplotlib.pyplot as plt
from tqdm import tqdm
import os



node_name = 'dual_robot_rl_real'
# ros_launch = ROSLauncher('dual_gazebo','view_dual_ur5e_model.launch')
# ros_launch.launch.start()

# rospy.loginfo("> STARTED Roslaunch-->" +
#                 str(ros_launch._launch_file_name))
rospy.init_node(node_name,
                anonymous=True, log_level=rospy.WARN)

ros_param_path = load_ros_params(rospackage_name="dual_robot_rl",
                    rel_path_from_package_to_file="config",
                    yaml_file_name="real/task_space_pick_and_place_real.yaml")

drl_state_publisher = rospy.Publisher("/dual_gym/drl/state", String, queue_size=10)
drl_action_publisher = rospy.Publisher("/dual_gym/drl/action", String, queue_size=10)
drl_reward_publisher = rospy.Publisher("/dual_gym/drl/reward", String, queue_size=10)
drl_ep_steps_publisher = rospy.Publisher("/dual_gym/drl/ep_steps", String, queue_size=10)
drl_train_steps_publisher = rospy.Publisher("/dual_gym/drl/train_steps", String, queue_size=10)

max_episode_steps = rospy.get_param("/dual_gym/rl/steps_per_episode", 200)

seed = rospy.get_param("/dual_gym/rand_seed", 3407)
torch.manual_seed(seed)
torch.cuda.manual_seed(seed)
torch.cuda.manual_seed_all(seed)

env = UR5PickAndPlaceEnv()
state_dim = env.observation_space['observation'].shape[0]+env.observation_space['desired_goal'].shape[0]+env.observation_space['achieved_goal'].shape[0]
action_dim = env.action_space.shape[0]

actor_lr = 3e-6
critic_lr = 1e-7
num_episodes = 50
hidden_dim = 256
gamma = 0.99999
lmbda = 0.95
entropy_coef = 0.01
epochs = 100
eps = 0.15
state_len = env.observation_space['observation'].shape[0]
achieved_goal_len = env.observation_space['achieved_goal'].shape[0]
device = torch.device("cuda") if torch.cuda.is_available() else torch.device(
    "mps")

agent = PPOContinuous(state_dim, hidden_dim, action_dim, actor_lr, critic_lr, lmbda, epochs, eps, gamma, device, entropy_coef)

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
            # transition_dict,her_info = her_process(transition_dict, state_len, achieved_goal_len,env.distance_threshold, info)
            trans_len = len(transition_dict['rewards'])
            # if her_info!='cant her':
            # trans_done = 1 if transition_dict['rewards'][-1] == 0 else 0
            agent.update(transition_dict)
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
                'trans_len':trans_len,
                # 'trans_done':trans_done,
                # 'her info':her_info,
                'episode':
                    '%d' % (num_episodes* i + i_episode + 1),
                'return':
                '%.3f' % np.mean(return_list[-10:]),
                "lr": agent.actor_optimizer.param_groups[0][
                            "lr"],
                "is success count": success_count,
                # "HER ratio":her_ratio
            })

            pbar.update(1)
    torch.save(agent.actor.state_dict(), os.path.join('/media/jt/Extreme SSD/Github/Dual_robot_real/src/dual_robot_rl',"model/ppo_dual_robot_pick_actor_%d.pkl" % i))
    # test_env  = PickPlace_UR5Env(sim_params, robot_params,visual_sensor_params)
    # evluation_policy(env=test_env, state_dim=agent.state_dim,
    #                  action_dim = agent.action_dim,
    #                  hidden_dim=agent.hidden_dim, 
    #                  device=agent.device,
    #                  model_num=i)
    # test_env.close()
    # del test_env
    # sim_params['is_train'] = True
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





