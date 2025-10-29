import numpy as np
import torch
from collections import defaultdict, deque
from torchvision import transforms
from PIL import Image
import cv2


def aggregate(data, method='max'):
    if isinstance(data[0], torch.Tensor):
        if method == 'max':
            # equivalent to any
            return torch.max(torch.stack(data))
        elif method == 'min':
            # equivalent to all
            return torch.min(torch.stack(data))
        elif method == 'mean':
            return torch.mean(torch.stack(data))
        elif method == 'sum':
            return torch.sum(torch.stack(data))
        else:
            raise NotImplementedError()
    else:
        if method == 'max':
            # equivalent to any
            return np.max(data)
        elif method == 'min':
            # equivalent to all
            return np.min(data)
        elif method == 'mean':
            return np.mean(data)
        elif method == 'sum':
            return np.sum(data)
        else:
            raise NotImplementedError()


def stack_last_n_obs(all_obs, n_steps):
    assert(len(all_obs) > 0)
    all_obs = list(all_obs)
    result = np.zeros((n_steps,) + all_obs[-1].shape, 
        dtype=all_obs[-1].dtype)
    start_idx = -min(n_steps, len(all_obs))
    result[start_idx:] = np.array(all_obs[start_idx:])
    if n_steps > len(all_obs):
        # pad
        result[:start_idx] = result[start_idx]
    return result

def obsdealimg(obs):
    # top_bbox = [87,80,500,400]
    # top_img = np.array(_transforms(Image.fromarray(cv2.cvtColor(obs['top_img'],cv2.COLOR_BGR2RGB)[top_bbox[1]:top_bbox[1]+top_bbox[3], top_bbox[0]:top_bbox[0]+top_bbox[2],:]))).transpose(2,0,1)
    # top_depth = np.array(_transforms(Image.fromarray(obs['top_depth'][top_bbox[1]:top_bbox[1]+top_bbox[3], top_bbox[0]:top_bbox[0]+top_bbox[2]],'L'))).reshape(-1,224,224)
    # right_bbox = find_yellow_region(obs['right_img'])
    # right_img = np.array(_transforms(Image.fromarray(cv2.cvtColor(obs['right_img'],cv2.COLOR_BGR2RGB)[right_bbox[1]:right_bbox[1]+right_bbox[3], right_bbox[0]:right_bbox[0]+right_bbox[2],:]))).transpose(2,0,1)
    # right_depth = np.array(_transforms(Image.fromarray(obs['right_depth'][right_bbox[1]:right_bbox[1]+right_bbox[3], right_bbox[0]:right_bbox[0]+right_bbox[2]],'L'))).reshape(-1,224,224)
    
    # obs['top_img'] = top_img.copy()
    # obs['top_depth'] = top_depth.copy()
    # obs['right_img'] = right_img.copy()
    # obs['right_depth'] = right_depth.copy()
    top_rgbd_tmp = np.concatenate([obs['top_img'], obs['top_depth'].reshape(obs['top_depth'].shape[0], obs['top_depth'].shape[1],1)], axis=-1).transpose([2, 0, 1])
    right_rgbd_tmp = np.concatenate([obs['right_img'], obs['right_depth'].reshape(obs['right_depth'].shape[0], obs['right_depth'].shape[1],1)], axis=-1).transpose([2, 0, 1])
    return {'top_rgbd':top_rgbd_tmp.astype(np.float32),
            'right_rgbd':right_rgbd_tmp.astype(np.float32),
            'agent_pos':obs['agent_pos'].astype(np.float32)}


class MultiStepWrapper(object):
    def __init__(self, 
            env, 
            n_obs_steps, 
            n_action_steps, 
            max_episode_steps=None,
        ):
        self.robot = env
        self.obs_keys = ['top_rgbd', 'right_rgbd', 'agent_pos']
        self.max_episode_steps = max_episode_steps
        self.n_obs_steps = n_obs_steps
        self.n_action_steps = n_action_steps

        self.obs = deque(maxlen=n_obs_steps+1)
        self.done = list()
        self.global_step = None
    
    def reset(self):
        """Resets the environment using kwargs."""
        obs = self.robot.reset()
        obs = obsdealimg(obs)

        self.obs = deque([obs], maxlen=self.n_obs_steps+1)
        self.global_step = 0

        obs = self._get_obs(self.n_obs_steps)
        return obs

    def step(self, action):
        """
        actions: (n_action_steps,) + action_shape
        """
        for act in action:
            if len(self.done) > 0 and self.done[-1]:
                # termination
                break
            obs = self.robot.step(act)
            observation = obsdealimg(obs)

            self.obs.append(observation)
            if (self.max_episode_steps is not None) and (self.global_step >= self.max_episode_steps):
                # truncation
                done = True
            else:
                done = False
            self.done.append(done)
            self.global_step+=1

        observation = self._get_obs(self.n_obs_steps)
        done = aggregate(self.done, 'max')
        return observation, done

    def _get_obs(self, n_steps=1):
        """
        Output (n_steps,) + obs_shape
        """
        assert(len(self.obs) > 0)

        result = dict()
        for key in self.obs_keys:
            result[key] = stack_last_n_obs(
                [obs[key] for obs in self.obs],
                n_steps
            )
        return result
