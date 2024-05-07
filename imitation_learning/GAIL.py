import sys
import os
current_directory = os.getcwd()
sys.path.append(current_directory)
import torch
import numpy as np
from tqdm import tqdm
import torch.nn.functional as F
from utilize import distance
import torch.nn as nn
import matplotlib.pyplot as plt
import random
import rl_utils
import collections

def compute_advantage(gamma, lmbda, td_delta):
    td_delta = td_delta.detach().numpy()
    advantage_list = []
    advantage = 0.0
    for delta in td_delta[::-1]:
        advantage = gamma * lmbda * advantage + delta
        advantage_list.append(advantage)
    advantage_list.reverse()
    return torch.tensor(np.array(advantage_list), dtype=torch.float)

class PolicyNet(torch.nn.Module):
    def __init__(self, state_dim, hidden_dim, action_dim):
        super(PolicyNet, self).__init__()
        self.fc1 = torch.nn.Linear(state_dim, hidden_dim)
        self.fc2 = torch.nn.Linear(hidden_dim, hidden_dim)
        self.fc_mu = torch.nn.Linear(hidden_dim, action_dim)
        self.fc_std = torch.nn.Linear(hidden_dim, action_dim)
        self.device = (
            torch.device("cuda") if torch.cuda.is_available() else torch.device("mps")
        )

    def forward(self, x):
        x = F.relu(self.fc1(x))
        x = F.relu(self.fc2(x))
        mu = torch.tanh(self.fc_mu(x))
        std = F.softplus(self.fc_std(x))
        std = std + 1e-8 * torch.ones(size=std.shape).to(self.device)
        return mu, std


class ValueNet(torch.nn.Module):
    def __init__(self, state_dim, hidden_dim):
        super(ValueNet, self).__init__()
        self.fc1 = torch.nn.Linear(state_dim, hidden_dim)
        self.fc2 = torch.nn.Linear(hidden_dim, hidden_dim)
        self.fc3 = torch.nn.Linear(hidden_dim, 1)
    def forward(self, x):
        x = F.relu(self.fc1(x))
        x = F.relu(self.fc2(x))

        return self.fc3(x)
class PPOContinuous:
    """处理连续动作的PPO算法"""

    def __init__(self, state_dim, hidden_dim, action_dim, actor_lr, critic_lr, lmbda, epochs, eps, gamma, device, entropy_coef):
        self.actor = PolicyNet(state_dim, hidden_dim, action_dim).to(
            device
        )
        self.critic = ValueNet(state_dim, hidden_dim).to(device)
        self.lr_a = actor_lr
        self.lr_c = critic_lr
        self.actor_optimizer = torch.optim.Adam(
            self.actor.parameters(), lr=actor_lr, eps=1e-5
        )
        self.critic_optimizer = torch.optim.Adam(
            self.critic.parameters(), lr=critic_lr, eps=1e-5
        )
        self.gamma = gamma
        self.lmbda = lmbda
        self.epochs = epochs
        self.eps = eps
        self.device = device
        self.entropy_coef = entropy_coef
        self.hidden_dim = hidden_dim
        self.state_dim = state_dim
        self.action_dim = action_dim

    def take_action(self, state):
        state = torch.tensor(state, dtype=torch.float).to(self.device)
        mu, sigma = self.actor(state)
        action_dist = torch.distributions.Normal(mu, sigma)
        action = action_dist.sample()
        action = action.clamp(-1.0, 1.0)
        return action.detach().cpu().numpy()

    def update(self, transition_dict):
        states = torch.tensor(transition_dict['states'],
                              dtype=torch.float).to(self.device)
        actions = torch.tensor(transition_dict['actions'],
                               dtype=torch.float).to(self.device)
        rewards = torch.tensor(transition_dict['rewards'],
                               dtype=torch.float).view(-1, 1).to(self.device)
        next_states = torch.tensor(transition_dict['next_states'],
                                   dtype=torch.float).to(self.device)
        dones = torch.tensor(transition_dict['dones'],
                             dtype=torch.float).view(-1, 1).to(self.device)
        td_target = rewards + self.gamma * self.critic(next_states) * (1 - dones)
        td_delta = td_target - self.critic(states)
        advantage = compute_advantage(self.gamma, self.lmbda, td_delta.cpu()).to(
            self.device
        )
        mu, std = self.actor(states)
        action_dists = torch.distributions.Normal(mu.detach(), std.detach())
        # 动作是正态分布
        old_log_probs = action_dists.log_prob(actions)

        updata_size = 10
        for _ in range(updata_size):
            mu, std = self.actor(states)
            action_dists = torch.distributions.Normal(mu, std)
            dist_entropy = action_dists.entropy() # 计算熵
            log_probs = action_dists.log_prob(actions)
            ratio = torch.exp(log_probs - old_log_probs)
            surr1 = ratio * advantage
            surr2 = torch.clamp(ratio, 1 - self.eps, 1 + self.eps) * advantage
            actor_loss = (
                -torch.min(surr1, surr2) - self.entropy_coef * dist_entropy
            )  # 计算actor的损失加入了熵
            critic_loss = torch.mean(
                F.mse_loss(self.critic(states), td_target.detach())
            )
            self.actor_optimizer.zero_grad()
            actor_loss.mean().backward()
            self.actor_optimizer.step()

            self.critic_optimizer.zero_grad()
            critic_loss.backward()
            self.critic_optimizer.step()

    def lr_decay(self, total_steps):
        lr_a_now = self.lr_a * (1 - total_steps / self.epochs)
        lr_c_now = self.lr_c * (1 - total_steps / self.epochs)
        for p in self.actor_optimizer.param_groups:
            p["lr"] = lr_a_now
        for p in self.critic_optimizer.param_groups:
            p["lr"] = lr_c_now




class Discriminator(nn.Module):
    def __init__(self, state_dim, hidden_dim, action_dim):
        super(Discriminator, self).__init__()
        self.fc1 = torch.nn.Linear(state_dim + action_dim, hidden_dim)
        self.fc2 = torch.nn.Linear(hidden_dim, hidden_dim)
        self.fc3 = torch.nn.Linear(hidden_dim, 1)

    def forward(self, x, a):
        cat = torch.cat([x, a], dim=1)
        x = F.relu(self.fc1(cat))
        x = F.relu(self.fc2(x))
        return torch.sigmoid(self.fc3(x))
    

class GAIL:
    def __init__(self, agent, state_dim, action_dim, hidden_dim, lr_d, device, epochs):
        self.device = device
        self.lr_d = lr_d
        self.discriminator = Discriminator(state_dim, hidden_dim,
                                           action_dim).to(self.device)
        self.discriminator_optimizer = torch.optim.Adam(
            self.discriminator.parameters(), lr=self.lr_d)
        self.agent = agent
        self.epochs = epochs

    def learn(self, expert_s, expert_a, agent_s, agent_a, next_s, dones):
        expert_states = torch.tensor(expert_s, dtype=torch.float).to(self.device)
        expert_actions = torch.tensor(expert_a, dtype=torch.float).to(self.device)
        agent_states = torch.tensor(agent_s, dtype=torch.float).to(self.device)
        agent_actions = torch.tensor(agent_a, dtype=torch.float).to(self.device)
        # expert_actions = F.one_hot(expert_actions, num_classes=2).float()
        # agent_actions = F.one_hot(agent_actions, num_classes=2).float()

        expert_prob = self.discriminator(expert_states, expert_actions)
        agent_prob = self.discriminator(agent_states, agent_actions)
        discriminator_loss = nn.BCELoss()(
            agent_prob, torch.ones_like(agent_prob)) + nn.BCELoss()(
                expert_prob, torch.zeros_like(expert_prob))
        self.discriminator_optimizer.zero_grad()
        discriminator_loss.backward()
        self.discriminator_optimizer.step()

        rewards = -torch.log(agent_prob).detach().cpu().numpy()
        transition_dict = {
            'states': agent_s,
            'actions': agent_a,
            'rewards': rewards,
            'next_states': next_s,
            'dones': dones
        }
        self.agent.update(transition_dict)
        return discriminator_loss.detach().cpu().numpy().copy(), np.sum(rewards.copy())
    def lr_decay(self, total_steps):
        lr_d_now = self.lr_d * (1 - total_steps / self.epochs)
        for p in self.discriminator_optimizer.param_groups:
            p["lr"] = lr_d_now



class ReplayBuffer_Trajectory:
    ''' 存储轨迹的经验回放池 '''
    def __init__(self, capacity, dis_threshold, use_her, batch_size,state_len,achieved_goal_len):
        self.buffer = collections.deque(maxlen=capacity)
        self.dis_threshold = dis_threshold
        self.use_her = use_her
        self.batch_size = batch_size
        self.state_len = state_len
        self.achieved_goal_len = achieved_goal_len
    def add_trajectory(self, trajectory):
        self.buffer.append(trajectory)

    def size(self):
        return len(self.buffer)

    def sample(self,her_ratio):
        batch = dict(states=[],
                     actions=[],
                     next_states=[],
                     rewards=[],
                     dones=[],
                     gamma_pow=[])
        for _ in range(self.batch_size):
            traj = random.sample(self.buffer, 1)[0]
            step_state = np.random.randint(traj.length)
            state = traj.states[step_state]
            next_state = traj.states[step_state + 1]
            action = traj.actions[step_state]
            reward = traj.rewards[step_state]
            done = traj.dones[step_state]
            gamma_pow = 0


            # if self.use_her and traj.rewards[-1] != 0:
            if self.use_her and np.random.uniform() <= her_ratio:
                step_goal = np.random.randint(step_state + 1, traj.length + 1)
                goal = traj.states[step_goal][self.state_len:self.state_len+self.achieved_goal_len].copy()   # 使用HER算法的future方案设置目标
                dis = distance(next_state[self.state_len:self.state_len+self.achieved_goal_len], goal)
                reward = -1.0 if dis > self.dis_threshold else 0
                done = False if dis > self.dis_threshold else True
                state = np.hstack((state[:self.state_len+self.achieved_goal_len], goal)).copy() 
                next_state = np.hstack((next_state[:self.state_len+self.achieved_goal_len], goal)).copy() 
                gamma_pow = step_goal-step_state

            batch['states'].append(state.copy())
            batch['next_states'].append(next_state.copy())
            batch['actions'].append(action.copy())
            batch['rewards'].append(reward)
            batch['dones'].append(done)
            batch['gamma_pow'].append(gamma_pow)

        batch['states'] = np.array(batch['states'])
        batch['next_states'] = np.array(batch['next_states'])
        batch['actions'] = np.array(batch['actions'])
        return batch