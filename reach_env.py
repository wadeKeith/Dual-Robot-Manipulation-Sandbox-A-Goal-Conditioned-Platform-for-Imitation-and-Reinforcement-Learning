from typing import Any, Dict, Tuple
import time

import numpy as np
from gymnasium import spaces

from pick_place_env import GoalObservationMixin
from ur5_robotiq import UR5Robotiq140
from utilize import connect_pybullet, distance, set_debug_camera


class ReachUR5Env(GoalObservationMixin):
    """
    Goal-conditioned reaching task for a single UR5e arm.
    Shares the same API conventions as the pick-and-place environments.
    """

    def __init__(self, sim_params, robot_params, visual_sensor_params):
        self.vis = sim_params["use_gui"]
        self._pb = connect_pybullet(sim_params["timestep"], show_gui=self.vis)
        self.SIMULATION_STEP_DELAY = sim_params["timestep"]
        self.control_type = sim_params["control_type"]
        self.gripper_enable = sim_params.get("gripper_enable", True)
        self.is_train = sim_params["is_train"]
        self.distance_threshold = sim_params["distance_threshold"]
        self.load_standard_environment()

        self.arm_gripper = UR5Robotiq140(self._pb, robot_params=robot_params, use_gui=self.vis)
        self.arm_gripper.step_simulation = self.step_simulation

        if self.vis:
            set_debug_camera(self._pb, visual_sensor_params)

        self.handle_pos = np.array([0.645, 0.0, 0.175], dtype=np.float32)
        self.goal_range_low = np.array([0, -0.2, -0.175], dtype=np.float32)
        self.goal_range_high = np.array([0.4, 0.2, 0.175], dtype=np.float32)

        if self.control_type == "joint" and self.gripper_enable:
            observation_bound_now = np.ones(shape=(self.arm_gripper.num_control_dofs,)) * np.pi
            observation_bound = np.concatenate([observation_bound_now, observation_bound_now])
        elif self.control_type == "joint" and not self.gripper_enable:
            observation_bound_now = np.ones(shape=(self.arm_gripper.arm_num_dofs,)) * np.pi
            observation_bound = np.concatenate([observation_bound_now, observation_bound_now])
        elif self.control_type == "end" and self.gripper_enable:
            observation_bound_now = np.concatenate(
                [np.array([2, 2, 2]), np.ones(shape=(self.arm_gripper.num_control_dofs - self.arm_gripper.arm_num_dofs,)) * np.pi]
            )
            observation_bound = np.concatenate([observation_bound_now, observation_bound_now])
        else:
            observation_bound_now = np.array([2, 2, 2])
            observation_bound = np.concatenate([observation_bound_now, observation_bound_now])

        observation_space = spaces.Box(-observation_bound, observation_bound, dtype=np.float32)
        achieved_goal_bound = np.array([2, 2, 2], dtype=np.float32)
        achieved_space = spaces.Box(-achieved_goal_bound, achieved_goal_bound, dtype=np.float32)
        desired_space = spaces.Box(-achieved_goal_bound, achieved_goal_bound, dtype=np.float32)
        self.observation_space = spaces.Dict(
            {
                "observation": observation_space,
                "achieved_goal": achieved_space,
                "desired_goal": desired_space,
            }
        )
        n_action = 3 if self.control_type == "end" else 6
        if self.gripper_enable:
            n_action += 1
        self.action_space = spaces.Box(low=-1, high=1, shape=(n_action,), dtype=np.float32)
        self.time = None
        self.time_limitation = 200
        self.goal = None
        self.n_sub_step = 50

    def reset(self, seed=None) -> Tuple[Dict[str, np.ndarray], Dict[str, Any]]:
        self.handle_pos = np.array([0.645, 0.0, 0.175], dtype=np.float32)
        self.time = 0
        self.arm_gripper.reset(self.gripper_enable)

        if self.is_train:
            self.goal = self._sample_goal().copy()
            if self.vis:
                self._pb.addUserDebugPoints(
                    pointPositions=[self.goal.copy()],
                    pointColorsRGB=[[255, 0, 0]],
                    pointSize=20,
                    lifeTime=self.time_limitation * self.SIMULATION_STEP_DELAY * self.n_sub_step,
                )
        else:
            self.goal = self.handle_pos
            if self.vis:
                self._pb.addUserDebugPoints(
                    pointPositions=[self.goal.copy()],
                    pointColorsRGB=[[255, 0, 0]],
                    pointSize=20,
                    lifeTime=self.time_limitation * self.SIMULATION_STEP_DELAY * self.n_sub_step,
                )

        robot_obs_old = self.arm_gripper.get_joint_obs(self.control_type, self.gripper_enable).astype(np.float32).copy()
        robot_obs_new = self.arm_gripper.get_joint_obs(self.control_type, self.gripper_enable).astype(np.float32)
        robot_obs = np.concatenate([robot_obs_old, robot_obs_new])
        obs_dict = self._get_obs(robot_obs)
        obs = self.dictobs2npobs(obs_dict, self.observation_space)
        info = {"is_success": bool(self.is_success(obs_dict["achieved_goal"], obs_dict["desired_goal"]))}
        return obs, info

    def step(self, action) -> Tuple[Dict[str, np.ndarray], float, bool, bool, Dict[str, Any]]:
        self.time += 1
        robot_obs_old = self.arm_gripper.get_joint_obs(self.control_type, self.gripper_enable).astype(np.float32).copy()
        if self.gripper_enable:
            self.arm_gripper.move_ee(action[:-1], self.control_type)
            self.arm_gripper.move_gripper(action[-1])
        else:
            self.arm_gripper.move_ee(action, self.control_type)
        self.step_simulation()
        robot_obs_new = self.arm_gripper.get_joint_obs(self.control_type, self.gripper_enable).astype(np.float32)
        robot_obs = np.concatenate([robot_obs_old, robot_obs_new])
        obs_dict = self._get_obs(robot_obs)
        obs = self.dictobs2npobs(obs_dict, self.observation_space)
        info = {"is_success": bool(self.is_success(obs_dict["achieved_goal"], obs_dict["desired_goal"]))}
        terminated = self.compute_terminated(obs_dict["achieved_goal"], obs_dict["desired_goal"], info)
        truncated = self.compute_truncated(obs_dict["achieved_goal"], obs_dict["desired_goal"], info)
        reward = float(self.compute_reward(obs_dict["achieved_goal"], obs_dict["desired_goal"], info))
        return obs, reward, terminated, truncated, info

    def compute_reward(self, achieved_goal, desired_goal, info: Dict[str, Any]) -> np.ndarray:
        d = distance(achieved_goal, desired_goal)
        return -np.array(d > self.distance_threshold, dtype=np.float32)

    def compute_terminated(self, achieved_goal, desired_goal, info) -> bool:
        d = distance(achieved_goal, desired_goal)
        return d < self.distance_threshold

    def compute_truncated(self, achieved_goal, desired_goal, info) -> bool:
        d = distance(achieved_goal, desired_goal)
        if d < self.distance_threshold:
            return True
        return self.time >= self.time_limitation

    def is_success(self, achieved_goal: np.ndarray, desired_goal: np.ndarray) -> np.ndarray:
        d = distance(achieved_goal, desired_goal)
        return np.array(d < self.distance_threshold, dtype=bool)

    def step_simulation(self):
        for _ in range(self.n_sub_step):
            self._pb.stepSimulation()
            if self.vis:
                time.sleep(self.SIMULATION_STEP_DELAY)

    def load_standard_environment(self):
        self._pb.loadURDF("./assets/plane/plane.urdf", [0, 0, -0.625])
        self._pb.loadURDF(
            "./assets/environment_objects/table/table.urdf",
            [0.50, 0.00, -0.625],
            [0.0, 0.0, 0.0, 1.0],
        )

    def _get_obs(self, robot_obs):
        achieved_goal = self.get_achieved_goal().astype(np.float32)
        desired_goal = self.goal.copy().astype(np.float32)
        return {"observation": robot_obs, "achieved_goal": achieved_goal, "desired_goal": desired_goal}

    def get_achieved_goal(self) -> np.ndarray:
        left_finger_pad_position = np.array(
            self._pb.getLinkState(self.arm_gripper.embodiment_id, self.arm_gripper.left_finger_pad_id)[4],
            dtype=np.float64,
        )
        right_finger_pad_position = np.array(
            self._pb.getLinkState(self.arm_gripper.embodiment_id, self.arm_gripper.right_finger_pad_id)[4],
            dtype=np.float64,
        )
        achieved_goal_finger_pos = (left_finger_pad_position + right_finger_pad_position) / 2
        return achieved_goal_finger_pos

    def _sample_goal(self) -> np.ndarray:
        goal = self.handle_pos.copy()
        noise = np.random.uniform(self.goal_range_low, self.goal_range_high)
        if np.random.random() < 0.3:
            noise[2] = 0.0
        goal += noise
        return goal

    def close(self):
        if self._pb.isConnected():
            self._pb.disconnect()

    def render(self, mode="human"):
        pass

    def seed(self, seed=None):
        pass
