from typing import Any, Dict, Optional, Tuple
import time
from ur5_robotiq import UR5Robotiq85
from utilize import connect_pybullet, set_debug_camera, Camera, distance
from gymnasium import spaces
import numpy as np
import math


class PickPlace_UR5Env(object):
    def __init__(self, sim_params, robot_params, visual_sensor_params):
        # super().__init__()
        self.vis = sim_params['use_gui']
        self._pb = connect_pybullet(sim_params['timestep'], show_gui=self.vis)
        self.SIMULATION_STEP_DELAY = sim_params['timestep']
        self.control_type = sim_params['control_type']
        self.is_train = sim_params['is_train']
        self.distance_threshold = sim_params['distance_threshold']
        self.load_standard_environment()

        # initialize a robot arm and gripper
        self.arm_gripper = UR5Robotiq85(
            self._pb,
            robot_params=robot_params,
            use_gui = self.vis,
        )
        self.arm_gripper.step_simulation = self.step_simulation
        # # Initialize the camera
        # self.camera = Camera(self._pb, visual_sensor_params)
        if self.vis:
            set_debug_camera(self._pb, visual_sensor_params)
        # Initialize the goal range
        self.blockUid = -1
        self.goal_range_low = np.array([0.2, -0.3, 0.04+0.725])
        self.goal_range_high = np.array([0.4, 0.3, 1])
        # rgb_obs_space = spaces.Box(low=0, high=255, shape=(visual_sensor_params['image_size'][0], visual_sensor_params['image_size'][1], 4), dtype=np.uint8)
        # depth_obs_space = spaces.Box(low=0, high=1, shape=(visual_sensor_params['image_size'][0], visual_sensor_params['image_size'][1]), dtype=np.float32)
        # seg_obs_space = spaces.Box(low=-1, high=255, shape=(visual_sensor_params['image_size'][0], visual_sensor_params['image_size'][1]), dtype=np.int32)
        if self.control_type=='joint':
            observation_bound_now = np.ones(shape=(self.arm_gripper.num_control_dofs,))*3.14159265359
            observation_bound = np.concatenate([observation_bound_now,observation_bound_now])
        elif self.control_type=='end':
            observation_bound = np.concatenate([np.ones(shape=(3,))*2,np.ones(shape=(3,))*2,
                                                np.ones(shape=(3,))*2,np.ones(shape=(3,))*2,
                                                np.ones(shape=(3,))*3.14159265359, np.ones(shape=(3,))*3.14159265359,
                                                np.ones(shape=(3,))*2, np.ones(shape=(3,))*2,
                                                np.ones(shape=(3,))*3.2, np.ones(shape=(3,))*3.2,
                                                np.ones(shape=(self.arm_gripper.num_control_dofs-self.arm_gripper.arm_num_dofs*self.arm_gripper.arm_num,))*3.14159265359,
                                                np.ones(shape=(3,))*3.14159265359,
                                                np.ones(shape=(3,))*2,np.ones(shape=(3,))*2])
            # observation_bound = np.concatenate([observation_bound_now,observation_bound_now])
        observation_space = spaces.Box(-observation_bound, observation_bound, dtype=np.float32)
        achieved_space = spaces.Box(np.array([0.2, -0.3, 0]), self.goal_range_high, dtype=np.float32)
        desired_space = spaces.Box(np.array([0.2, -0.3, 0]), self.goal_range_high, dtype=np.float32)
        # self.observation_space = spaces.Dict({
        #     'rgb': rgb_obs_space,
        #     'depth': depth_obs_space,
        #     'seg': seg_obs_space,
        #     'positions': positions_obs_space,
        #     'velocities': velocities_obs_space,
        #     'finger_pos': ee_pos_obs_space
        # })
        self.observation_space = spaces.Dict({
            'observation': observation_space,
            'achieved_goal': achieved_space,
            'desired_goal': desired_space,
        })
        n_action = 4*2 if self.control_type == "end" else 7*2  # control (x, y z) if "ee", else, control the 7 joints
        self.action_space = spaces.Box(low=-1, high=1, shape=(n_action,),dtype=np.float32)
        self.time = None
        self.time_limitation = 100
        self.n_sub_step = 50
    #     self.xin_left = self._pb.addUserDebugParameter("x_l", -1, 1, 0)
    #     self.yin_left = self._pb.addUserDebugParameter("y_l", -1, 1, 0)
    #     self.zin_left = self._pb.addUserDebugParameter("z_l", -1, 1, 0)
    #     self.xin_right = self._pb.addUserDebugParameter("x_r", -1, 1, 0)
    #     self.yin_right = self._pb.addUserDebugParameter("y_r", -1, 1, 0)
    #     self.zin_right = self._pb.addUserDebugParameter("z_r", -1, 1, 0)
    #     self.gripper_opening_length_contro_left = self._pb.addUserDebugParameter("gripper_l", -1, 1, 0)
    #     self.gripper_opening_length_contro_right = self._pb.addUserDebugParameter("gripper_r", -1, 1, 0)

        

    # def read_debug_parameter(self):
    #     # read the value of task parameter
    #     x_l = self._pb.readUserDebugParameter(self.xin_left)
    #     y_l = self._pb.readUserDebugParameter(self.yin_left)
    #     z_l = self._pb.readUserDebugParameter(self.zin_left)
    #     x_r = self._pb.readUserDebugParameter(self.xin_right)
    #     y_r = self._pb.readUserDebugParameter(self.yin_right)
    #     z_r = self._pb.readUserDebugParameter(self.zin_right)
    #     gripper_opening_length_l = self._pb.readUserDebugParameter(self.gripper_opening_length_contro_left)
    #     gripper_opening_length_r = self._pb.readUserDebugParameter(self.gripper_opening_length_contro_right)

    #     return x_l, y_l, z_l, x_r, y_r, z_r, gripper_opening_length_l, gripper_opening_length_r

    def reset(self,seed=None) -> Tuple[Dict[str, np.ndarray], Dict[str, Any]]:
        """
        Reset the pose of the arm and sensor
        """
        self.time = 0
        self.goal = None
        self.goal_ang = None
        self.achieved_goal_initial = None
        self.achieved_goal_initial_ang = None
        
        self.arm_gripper.reset()
        # block: x in (0.6, 1), y in (-0.4, 0.4), z = 0.04+0.725
        # target: x in (0.6, 1), y in (-0.4, 0.4), z in (0.04, 0.4)
        if self.is_train:
            initial_done = False
            while not initial_done:
                achieved_goal_initial,achieved_goal_initial_ang = self._sample_achieved_goal_initial()
                goal,goal_ang = self._sample_goal()
                initial_done = False if distance(achieved_goal_initial, goal) <= self.distance_threshold else True
            self.achieved_goal_initial = achieved_goal_initial.copy()
            self.achieved_goal_initial_ang = achieved_goal_initial_ang
            self.goal = goal.copy()
            self.goal_ang = goal_ang
        else:
            self.achieved_goal_initial = np.array([0.3,0.15,0.04+0.725])
            self.achieved_goal_initial_ang = 0
            self.goal = np.array([0.35,-0.15,0.9])
            self.goal_ang = 0
        if self.blockUid == -1:
            self.blockUid = self._pb.loadURDF("./assets/urdf/cube_small_pick.urdf", self.achieved_goal_initial,
                                        self._pb.getQuaternionFromEuler([0,0,self.achieved_goal_initial_ang]))
            self.targetUid = self._pb.loadURDF("./assets/urdf/cube_small_target_pick.urdf",
                                        self.goal,
                                        self._pb.getQuaternionFromEuler([0,0,self.goal_ang]), useFixedBase=1)
        else:
            self._pb.removeBody(self.blockUid)
            self._pb.removeBody(self.targetUid)
            self.blockUid = self._pb.loadURDF("./assets/urdf/cube_small_pick.urdf", self.achieved_goal_initial,
                                        self._pb.getQuaternionFromEuler([0,0,self.achieved_goal_initial_ang]))
            self.targetUid = self._pb.loadURDF("./assets/urdf/cube_small_target_pick.urdf",
                                        self.goal,
                                        self._pb.getQuaternionFromEuler([0,0,self.goal_ang]), useFixedBase=1)
        self._pb.setCollisionFilterPair(self.targetUid, self.blockUid, -1, -1, 0)
        robot_obs = self.arm_gripper.get_joint_obs(self.control_type).copy()
        # robot_obs = np.concatenate([robot_obs_old, robot_obs_new])
        obs_dict = self._get_obs(robot_obs)
        obs = self.dictobs2npobs(obs_dict, self.observation_space)
        info = {"is_success": bool(self.is_success(obs_dict["achieved_goal"], obs_dict['desired_goal']))}
        return (obs, info,obs_dict)

    def step(self, action) -> Tuple[Dict[str, np.ndarray], float, bool, bool, Dict[str, Any]]:
        """
        action: (x, y, z, roll, pitch, yaw, gripper_opening_length) for End Effector Position Control
                (a1, a2, a3, a4, a5, a6, a7, gripper_opening_length) for Joint Position Control
        control_method:  'end' for end effector position control
                         'joint' for joint position control
        """
        self.time +=1
        assert self.control_type in ('joint', 'end')
        self.arm_gripper.move_ee(action[:-2], self.control_type)
        self.arm_gripper.move_gripper(action[-2:])
        self.step_simulation()
        robot_obs = self.arm_gripper.get_joint_obs(self.control_type).copy()
        # robot_obs = np.concatenate([robot_obs_old, robot_obs_new])
        obs_dict = self._get_obs(robot_obs)
        obs = self.dictobs2npobs(obs_dict, self.observation_space)
        info = {"is_success": bool(self.is_success(obs_dict['achieved_goal'], obs_dict['desired_goal']))}
        terminated = self.compute_terminated(obs_dict['achieved_goal'], obs_dict['desired_goal'], info)
        truncated = self.compute_truncated(obs_dict['achieved_goal'], obs_dict['desired_goal'], info)
        reward  = float(self.compute_reward(obs_dict['achieved_goal'], obs_dict['desired_goal'], info))
        
        return obs, reward, terminated, truncated, info,obs_dict

    def compute_reward(self, achieved_goal: np.ndarray, desired_goal: np.ndarray, info: Dict[str, Any]) -> np.ndarray:
        d = distance(achieved_goal, desired_goal)
        return -np.array(d > self.distance_threshold, dtype=np.float32)
    
    def compute_terminated(self, achieved_goal, desired_goal, info) -> bool:
        d = distance(achieved_goal, desired_goal)
        return d <= self.distance_threshold
    
    def compute_truncated(self, achieved_goal, desired_goal, info) -> bool:
        d = distance(achieved_goal, desired_goal)
        if d <= self.distance_threshold:
            return True
        else:
            if self.time >=self.time_limitation:
                return True
            else:
                return False

    def is_success(self, achieved_goal: np.ndarray, desired_goal: np.ndarray) -> np.ndarray:
        d = distance(achieved_goal, desired_goal)
        return np.array(d <= self.distance_threshold, dtype=bool)
    
    
    def step_simulation(self):
        """
        Hook p.stepSimulation()
        """
        for _ in range(self.n_sub_step):
            self._pb.stepSimulation()
            if self.vis:
                time.sleep(self.SIMULATION_STEP_DELAY)

    def load_standard_environment(self):
        """
        Load a standard environment with a plane and a table.
        """
        self._pb.loadURDF(
            "./assets/plane/plane.urdf",
            [0, 0, 0],
        )

    def _get_obs(self, robot_obs):
        achieved_goal,achieved_goal_orn = self.get_achieved_goal()
        desired_goal = self.goal.copy()
        left_arm_relative_pos = achieved_goal-robot_obs[:3].copy()
        right_arm_relative_pos = achieved_goal-robot_obs[3:6].copy()
        total_obs = np.concatenate([robot_obs.copy(),achieved_goal_orn.copy(),left_arm_relative_pos.copy(), right_arm_relative_pos.copy()])
        return {
            'observation': total_obs,
            'achieved_goal': achieved_goal,
            'desired_goal': desired_goal,
        }
    def dictobs2npobs(self,observation, dic_observation_space):
        list_obs = []
        for key in observation.keys():
            list_obs += ((observation[key]-dic_observation_space[key].low)/(dic_observation_space[key].high-dic_observation_space[key].low)).tolist()
        return np.array(list_obs)


    def get_achieved_goal(self) -> np.ndarray:
        achieved_goal_pos,achieved_goal_orn_qua = self._pb.getBasePositionAndOrientation(self.blockUid)
        achieved_goal_orn = self._pb.getEulerFromQuaternion(achieved_goal_orn_qua)
        # self._pb.addUserDebugPoints(pointPositions = [achieved_goal_pos], pointColorsRGB = [[0, 0, 255]], pointSize= 20, lifeTime= 0)
        return np.array(achieved_goal_pos), np.array(achieved_goal_orn)
    def _sample_goal(self) -> np.ndarray:
        """Sample a goal."""
        goal = np.random.uniform(self.goal_range_low, self.goal_range_high)
        goal_ang = np.random.uniform(-math.pi, math.pi)
        if np.random.random() < 0.3:
            goal[2] = self.goal_range_low[-1]
        return goal,goal_ang

    def _sample_achieved_goal_initial(self):
        achieved_goal_inital_xy = np.random.uniform(self.goal_range_low[:2], self.goal_range_high[:2])
        achieved_goal_inital = np.concatenate([achieved_goal_inital_xy,np.array(self.goal_range_low[-1]).reshape(1,)])
        achieved_goal_inital_ang = np.random.uniform(-math.pi, math.pi)
        return achieved_goal_inital,achieved_goal_inital_ang

    def close(self):
        if self._pb.isConnected():
            self._pb.disconnect()
    def render(self, mode='human'):
        pass
    def seed(self, seed=None):
        pass
