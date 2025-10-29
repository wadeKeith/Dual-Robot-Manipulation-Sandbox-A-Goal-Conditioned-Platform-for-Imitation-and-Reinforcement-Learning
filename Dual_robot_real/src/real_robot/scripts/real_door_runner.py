import numpy as np
import torch
import tqdm
import rospy
from multistep_wrapper import MultiStepWrapper
from real_robot_right import Robot_right
from policy.pytorch_util import dict_apply

class RealDoorRunner(object):
    def __init__(self,
                 max_steps=200,
                 n_obs_steps=2,
                 n_action_steps=4,
                 ):

        def env_fn():
            return MultiStepWrapper( Robot_right() ,
                n_obs_steps=n_obs_steps,
                n_action_steps=n_action_steps,
                max_episode_steps=max_steps,
            )
            # return MultiStepWrapper(
            #     SimpleVideoRecordingWrapper(
            #         env=AdroitEnv(env_name=task_name, use_point_cloud=False)
            #         ),
            #     n_obs_steps=n_obs_steps,
            #     n_action_steps=n_action_steps,
            #     max_episode_steps=max_steps,
            #     reward_agg_method='sum',
            # )

        self.env = env_fn()

        self.n_obs_steps = n_obs_steps
        self.n_action_steps = n_action_steps
        self.max_steps = max_steps

    def run(self, policy):
        device = policy.device
        env = self.env

        obs = env.reset()
        # policy.reset()

        done = False
        actual_step_count = 0
        while not done:
            # create obs dict
            np_obs_dict = dict(obs)
            # device transfer
            obs_dict = dict_apply(np_obs_dict,
                                    lambda x: torch.from_numpy(x).to(
                                        device=device))

            # run policy
            with torch.no_grad():
                obs_dict_input = {}  # flush unused keys
                obs_dict_input['top_rgbd'] = obs_dict['top_rgbd'].unsqueeze(0)
                # obs_dict_input['top_depth'] = obs_dict['top_depth'].unsqueeze(0)
                obs_dict_input['right_rgbd'] = obs_dict['right_rgbd'].unsqueeze(0)
                # obs_dict_input['right_depth'] = obs_dict['right_depth'].unsqueeze(0)
                obs_dict_input['agent_pos'] = obs_dict['agent_pos'].unsqueeze(0)
                
                action_dict = policy.predict_action(obs_dict_input)
                

            # device_transfer
            np_action_dict = dict_apply(action_dict,
                                        lambda x: x.detach().to('cpu').numpy())

            action = np_action_dict['action'].squeeze(0)
            # step env
            obs, done = env.step(action)
            done = np.all(done)
            actual_step_count += 1
        print(actual_step_count)
        self.env.robot.right_arm.stop_service_client()
        rospy.sleep(0.5)


