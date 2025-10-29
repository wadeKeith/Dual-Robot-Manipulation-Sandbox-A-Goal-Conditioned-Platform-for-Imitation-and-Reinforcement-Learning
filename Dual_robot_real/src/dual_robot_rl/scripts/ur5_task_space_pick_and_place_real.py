import datetime
import rospy
import numpy as np
import sys
from gymnasium import spaces
import rospkg
import math

from controller_py import transformations, spalg
import utils.cost_utils as cost
import ur5_goal_env
from utils.robot_env_utils import load_param_vars, save_log, randomize_initial_pose, apply_workspace_contraints
from gazebo_py.gazebo_spawner import delete_gazebo_models,GazeboModels
from gazebo_py.basic_models import BOX, BOX_TARGET
from gazebo_py.model import Model

from gazebo_msgs.msg import ModelState
from gazebo_grasp_plugin_ros.msg import GazeboGraspEvent
from gazebo_msgs.srv import GetModelState
from control_msgs.msg import JointControllerState
from std_msgs.msg import String
from visualization_msgs.msg import Marker
from geometry_msgs.msg import PoseStamped
import tf

# import rviz_tools_py as rviz_tools


def goal_distance(goal_a, goal_b):
        return np.linalg.norm(goal_a - goal_b, axis=-1)


class UR5PickAndPlaceEnv(ur5_goal_env.UR5Env):
    def __init__(self):
        self.get_robot_params()

        ur5_goal_env.UR5Env.__init__(self, self.driver)

        self._left_previous_joints = None
        self._right_previous_joints = None
        self.body_desk = None
        self.start_model = None
        self.target_model = None
        self.reward_per_step = []
        self.obs_per_step = []
        self.cube_range_low = [0, -0.35, 0.025+0.73]
        self.cube_range_high = [0.55, 0.35, 0.025+0.73] 
        self.cube_target_range_low = [0, -0.35, 0.025+0.73]
        self.cube_target_range_high = [0.55, 0.35, 0.4+0.73]
        self.workspace_low = [0,-1, 0.75]
        self.workspace_high = [0.7, 1, 1.73]
        self.cube_pose = None
        self.cube_target_pose = None
        self.achieved_goal = None
        self.goal = None
        self.action_factor = 0.001 #0.001
        # self.max_dist = None
        # self.action_result = None

        # self.left_ee_position_publisher = rospy.Publisher("/dual_gym/left_ee_position", String, queue_size=10)
        # self.right_ee_position_publisher = rospy.Publisher("/dual_gym/right_ee_position", String, queue_size=10)

        self.spawner = GazeboModels('dual_gazebo')
        # cube range is [0, -0.35, 0.025] ~ [0.55, 0.35, 0.025] 
        # self.start_model = Model("cube", [0.4, 0.2, 0.025], [0, 0, 0], file_type='string', string_model=BOX, reference_frame="desk_base_link")
        # cube target range is [0, -0.35, 0.025] ~ [0.55, 0.35, 0.4]
        # self.target_model = Model("cube_target", [0, 0.35, 0.2], [0, 0, 1.57/2], file_type='string', string_model=BOX_TARGET, reference_frame="desk_base_link")
        
        # self.spawner.load_models([self.body_desk, self.start_model, self.target_model])

        self.left_gripper_attached = False
        self.right_gripper_attached = False
        self.left_gripper_state_sub = rospy.Subscriber('/left/gazebo_grasp_plugin_event_republisher/grasp_events', 
                                                  GazeboGraspEvent, 
                                                  self.left_gripper_state, 
                                                  queue_size=10)
        self.right_gripper_state_sub = rospy.Subscriber('/right/gazebo_grasp_plugin_event_republisher/grasp_events', 
                                                  GazeboGraspEvent, 
                                                  self.right_gripper_state, 
                                                  queue_size=10)
        
        # self.cube_pose_publisher = rospy.Publisher('/cube_pose', PoseStamped, queue_size=10)
        
        
        # self.last_actions = np.zeros(self.n_actions)
        # obs = self._get_obs()

        self.obs_low = [0,0,
                        *self.workspace_low,
                        *self.workspace_low,
                        -2*math.pi,-2*math.pi,-math.pi,-2*math.pi,-2*math.pi,-2*math.pi,
                        -2*math.pi,-2*math.pi,-math.pi,-2*math.pi,-2*math.pi,-2*math.pi,
                        -1,-1,-1,
                        -1,-1,-1,
                        -1,-1,-1,
                        -1,-1,-1,
                        *self.workspace_low,
                        *self.workspace_low]
        
        self.obs_high = [1,1,
                        *self.workspace_high,
                        *self.workspace_high,
                        2*math.pi,2*math.pi,math.pi,2*math.pi,2*math.pi,2*math.pi,
                        2*math.pi,2*math.pi,math.pi,2*math.pi,2*math.pi,2*math.pi,
                        1,1,1,
                        1,1,1,
                        1,1,1,
                        1,1,1,
                        *self.workspace_high,
                        *self.workspace_high]

        self.reward_threshold = 500.0
        # self.n_count = 0

        self.action_space = spaces.Box(-1., 1.,
                                       shape=(self.n_actions, ),
                                       dtype='float32')

        self.observation_space = spaces.Dict(dict(
            observation=spaces.Box(np.array(self.obs_low), np.array(self.obs_high), shape=(len(self.obs_low),), dtype='float32'),
            achieved_goal=spaces.Box(np.array(self.workspace_low), np.array(self.workspace_high), shape=(3,), dtype='float32'),
            desired_goal=spaces.Box(np.array(self.workspace_low), np.array(self.workspace_high), shape=(3,), dtype='float32'),
        ))

        # self.trials = 1
        self.rate = rospy.Rate(1/self.agent_control_dt)

    def _init_env_variables(self):
        self.step_count = 0

    def get_model_state_client(self, model_name, relative_entity_name):
        rospy.wait_for_service('/gazebo/get_model_state')
        get_model_state = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
        return get_model_state(model_name, relative_entity_name)

    
    def get_cube_pose(self):
        response = self.get_model_state_client('cube_tmp', 'world')
        if response:    
            # response = self.transform_to_base_link(response)

            # transformed_pose_stamped = PoseStamped()
            # transformed_pose_stamped.header.frame_id = 'world'
            # transformed_pose_stamped.pose = response.pose

            # transformed_pose_stamped.header.stamp = rospy.Time.now()
            # self.cube_pose_publisher.publish(transformed_pose_stamped)
            
            return response.pose
        
    def get_cube_target_pose(self):
        response = self.get_model_state_client('cube_target_tmp', 'world')
        if response:    
            # response = self.transform_to_base_link(response)

            # transformed_pose_stamped = PoseStamped()
            # transformed_pose_stamped.header.frame_id = 'world'
            # transformed_pose_stamped.pose = response.pose

            # transformed_pose_stamped.header.stamp = rospy.Time.now()
            # self.cube_pose_publisher.publish(transformed_pose_stamped)
            
            return response.pose

    def left_gripper_state(self, msg): 
        self.left_gripper_attached = msg.attached
        return self.left_gripper_attached
    
    def right_gripper_state(self, msg): 
        self.right_gripper_attached = msg.attached
        return self.right_gripper_attached

    def scale_gripper(self, grip_cmd):
        return -0.804/2*(grip_cmd-1)
    

    def get_robot_params(self):
        prefix = "dual_gym"         
        load_param_vars(self, prefix)

        self.param_use_gazebo = True
        if self.driver == "robot":
            self.param_use_gazebo = False
        self.rand_init_counter = 0

    def set_init_joint_pose(self):
        left_qc = self.left_init_q
        right_qc = self.right_init_q
        self.left_arm.set_joint_positions(position=left_qc,
                                            wait=True,
                                            t=self.reset_time)
        self.right_arm.set_joint_positions(position=right_qc,
                                            wait=True,
                                            t=self.reset_time)
        left_gripper_cmd = self.scale_gripper(1)
        right_gripper_cmd = self.scale_gripper(1)
        self.left_gripper.set_joint_positions([left_gripper_cmd],wait=True,
                                        t=self.reset_time)
        self.right_gripper.set_joint_positions([right_gripper_cmd],wait=True,
                                        t=self.reset_time)
        rospy.Rate(1).sleep()


    def _randomize_cube_init_pose(self):
        initial_done = False
        while not initial_done:
            cube_inital,cube_inital_ang = self._sample_achieved_goal_initial()
            cube_target,cube_target_ang = self._sample_goal()
            initial_done = False if goal_distance(cube_inital, cube_target) <= self.distance_threshold else True
        return cube_inital, cube_inital_ang, cube_target, cube_target_ang


    def _sample_goal(self) -> np.ndarray:
        """Sample a goal."""
        cube_target = np.random.uniform(self.cube_target_range_low, self.cube_target_range_high)
        cube_target_ang = np.random.uniform(-math.pi, math.pi)
        if np.random.random() < 0.3:
            cube_target[2] = self.cube_target_range_low[-1]
        return cube_target,cube_target_ang

    def _sample_achieved_goal_initial(self):
        cube_inital_xy = np.random.uniform(self.cube_range_low[:2], self.cube_range_high[:2])
        cube_inital = np.concatenate([cube_inital_xy, np.array(self.cube_range_low[-1]).reshape(1,)])
        cube_inital_ang = np.random.uniform(-math.pi, math.pi)
        return cube_inital,cube_inital_ang

    def _set_init_pose(self):
        """Sets the Robot in its init pose
        """
        # self._log()
        cube_inital, cube_inital_ang, cube_target, cube_target_ang = self._randomize_cube_init_pose()
        if self.body_desk is None:
            self.set_init_joint_pose()
            self.body_desk = Model(name='body_desk',position=[0,0,0], orientation=[0,0,0,1],file_type='sdf', reference_frame='world')
            self.start_model = Model("cube", list(cube_inital), [0,0,cube_inital_ang], file_type='string', string_model= BOX, reference_frame="world")
            self.target_model = Model("cube_target", list(cube_target), [0,0,cube_target_ang], file_type='string', string_model=BOX_TARGET, reference_frame="world")
            self.spawner.load_models([self.body_desk, self.start_model, self.target_model])
        else:
            delete_gazebo_models([self.start_model.model_id, self.target_model.model_id, self.body_desk.model_id])
            self.set_init_joint_pose()
            self.start_model.set_pose(list(cube_inital),[0,0,cube_inital_ang])
            self.target_model.set_pose(list(cube_target),[0,0,cube_target_ang])
            self.spawner.load_models([self.body_desk,self.start_model,self.target_model])

        
           

    def get_points_and_vels(self, left_joint_angles, right_joint_angles):
        if self._left_previous_joints is None:
            self._left_previous_joints = self.left_arm.joint_angles()
        if self._right_previous_joints is None:
            self._right_previous_joints = self.right_arm.joint_angles()

        left_ee_pos_now = self.left_arm.end_effector(joint_angles=left_joint_angles, tip_link=self.left_arm.ee_link)
        right_ee_pos_now = self.right_arm.end_effector(joint_angles=right_joint_angles, tip_link=self.right_arm.ee_link)

        left_ee_pos_last = self.left_arm.end_effector(joint_angles=self._left_previous_joints, tip_link=self.left_arm.ee_link)
        right_ee_pos_last = self.right_arm.end_effector(joint_angles=self._right_previous_joints, tip_link=self.right_arm.ee_link)
        self._left_previous_joints = left_joint_angles 
        self._right_previous_joints = right_joint_angles

        left_linear_velocity = (left_ee_pos_now[:3] - left_ee_pos_last[:3]) / self.agent_control_dt
        right_linear_velocity = (right_ee_pos_now[:3] - right_ee_pos_last[:3]) / self.agent_control_dt
        left_angular_velocity = transformations.angular_velocity_from_quaternions(
            left_ee_pos_now[3:], left_ee_pos_last[3:], self.agent_control_dt)
        right_angular_velocity = transformations.angular_velocity_from_quaternions(
            right_ee_pos_now[3:], right_ee_pos_last[3:], self.agent_control_dt)
        left_velocity = np.concatenate((left_linear_velocity, left_angular_velocity))
        right_velocity = np.concatenate((right_linear_velocity, right_angular_velocity))

        left_error = self.cube_pose - left_ee_pos_now[:3]
        right_error = self.cube_pose - right_ee_pos_now[:3]

        return left_error, right_error, left_velocity, right_velocity
    
    def _get_obs(self):
        left_joint_angles = self.left_arm.joint_angles()
        right_joint_angles = self.right_arm.joint_angles()
        left_ee_pose = self.left_arm.end_effector(joint_angles=left_joint_angles, tip_link=self.left_arm.ee_link)
        right_ee_pose = self.right_arm.end_effector(joint_angles=right_joint_angles, tip_link=self.right_arm.ee_link)

        cube_pose = self.get_cube_pose()
        cube_position = np.array([cube_pose.position.x, cube_pose.position.y, cube_pose.position.z])
        cube_orientation = np.array([cube_pose.orientation.x, cube_pose.orientation.y, cube_pose.orientation.z, cube_pose.orientation.w])
        cube_pose = np.concatenate((cube_position, cube_orientation))

        cube_target_pose = self.get_cube_target_pose()
        cube_target_position = np.array([cube_target_pose.position.x, cube_target_pose.position.y, cube_target_pose.position.z])
        cube_target_orientation = np.array([cube_target_pose.orientation.x, cube_target_pose.orientation.y, cube_target_pose.orientation.z, cube_target_pose.orientation.w])
        cube_target_pose = np.concatenate((cube_target_position, cube_target_orientation))

        self.cube_pose = cube_pose[:3].copy()
        self.cube_target_pose = cube_target_pose[:3].copy()

        if min(goal_distance(self.cube_pose, left_ee_pose[:3]),goal_distance(self.cube_pose,right_ee_pose[:3])) < self.distance_threshold:
            print('achieved reach task')
            self.achieved_goal = self.cube_pose.copy()
            self.goal = self.cube_target_pose.copy()
        else:
            self.achieved_goal = left_ee_pose[:3].copy().astype(dtype=np.float32) if goal_distance(self.cube_pose, left_ee_pose[:3])<goal_distance(self.cube_pose,right_ee_pose[:3]) else right_ee_pose[:3].copy().astype(dtype=np.float32)
            self.goal = self.cube_pose.copy()

        left_error, right_error, left_velocity, right_velocity = self.get_points_and_vels(left_joint_angles, right_joint_angles)

        
        

        obs = np.concatenate([
            np.array(int(self.left_gripper_attached), dtype=np.float32).ravel(),
            np.array(int(self.right_gripper_attached), dtype=np.float32).ravel(),
            left_ee_pose.ravel()[:3],
            right_ee_pose.ravel()[:3],
            left_joint_angles, 
            right_joint_angles,
            left_error.ravel()[:3],
            right_error.ravel()[:3],
            left_velocity.ravel()[:3],
            right_velocity.ravel()[:3],
            self.cube_pose,
            self.cube_target_pose
            ], dtype=np.float32)
    

        
        
        return {
            'observation': obs.copy().astype(dtype=np.float32),
            'achieved_goal': self.achieved_goal.copy().astype(dtype=np.float32),
            'desired_goal': self.goal.copy().astype(dtype=np.float32),
            }
    
    def dictobs2npobs(self,observation, dic_observation_space):
        list_obs = []
        for key in observation.keys():
            list_obs += ((observation[key]-dic_observation_space[key].low)/(dic_observation_space[key].high-dic_observation_space[key].low)).tolist()
        return np.array(list_obs)

    def compute_reward(self, achieved_goal, goal, info):
        # Compute distance between goal and the achieved goal.
        d = goal_distance(achieved_goal, goal)
        f = goal_distance(self.cube_pose, self.cube_target_pose)
        g_left = 50 if self.left_gripper_attached else 0
        g_right = 50 if self.right_gripper_attached else 0
        if self.reward_type == 'sparse':
            return -(d > self.distance_threshold).astype(np.float32)
        elif self.reward_type == 'very_sparse':
            if d < self.distance_threshold:
                return 1.
            else:
                return 0.
        elif self.reward_type == 'dense':
            return -d + -f + g_left+g_right
             

    def _set_action(self, action):
        assert action.shape == (self.n_actions, )
        if np.any(np.isnan(action)):
            rospy.logerr("Invalid NAN action(s)" + str(action))
            sys.exit()
        assert np.all(action >= -1.0001) and np.all(action <= 1.0001)

        actions = np.copy(action)

        if self.n_actions == 3:
            cpose = self.ur5_arm.end_effector(joint_angles=None, tip_link=self.ur5_arm.ee_link)
            cmd = self.ur5_arm.end_effector(joint_angles=None, tip_link=self.ur5_arm.ee_link)
            delta = actions * 0.001
            delta = np.concatenate((delta[:3], [0, 0, 0])) # Do not change ax, ay, and az
            cmd = transformations.pose_euler_to_quaternion(cpose, delta)

        if self.n_actions == 8:
            # with gripperend_effector
            left_cpose = self.left_arm.end_effector(joint_angles=None, tip_link=self.left_arm.ee_link)
            right_cpose = self.right_arm.end_effector(joint_angles=None, tip_link=self.right_arm.ee_link)
            left_delta = actions[:3] * self.action_factor
            right_delta = actions[3:6] * self.action_factor
            left_delta = np.concatenate((left_delta[:3], [0, 0, 0])) # Do not change ax, ay, and az
            right_delta = np.concatenate((right_delta[:3], [0, 0, 0])) # Do not change ax, ay, and az
            left_cmd = transformations.pose_euler_to_quaternion(pose=left_cpose, 
                                                                delta=left_delta,
                                                                workspace_low=self.workspace_low, 
                                                                workspace_high=self.workspace_high)
            right_cmd = transformations.pose_euler_to_quaternion(pose=right_cpose, 
                                                                 delta=right_delta, 
                                                                 workspace_low=self.workspace_low,
                                                                 workspace_high=self.workspace_high)
            left_gripper_cmd = self.scale_gripper(actions[6])
            right_gripper_cmd = self.scale_gripper(actions[7])
            

        if self.n_actions == 6:
            cpose = self.ur5_arm.end_effector(joint_angles=None, tip_link=self.ur5_arm.ee_link)
            delta = actions * 0.001
            cmd = transformations.pose_euler_to_quaternion(cpose, delta)
       
        self.left_arm.set_target_pose_flex(left_cmd, t=self.agent_control_dt)
        self.right_arm.set_target_pose_flex(right_cmd, t = self.agent_control_dt)
        self.left_gripper._flexible_trajectory([left_gripper_cmd], time=self.agent_control_dt)
        self.right_gripper._flexible_trajectory([right_gripper_cmd], time=self.agent_control_dt)


        # gripper_cmd = self.scale_gripper(-1.)
        # self.ur5_gripper.gripper_joint_control(gripper_cmd)
        # cube_pose = self.get_cube_pose()
        # cube_position = np.array([cube_pose.position.x, cube_pose.position.y, cube_pose.position.z+0.02])
        # cube_pose = np.concatenate((cube_position, cpose[3:]))
        # cmd = transformations.pose_euler_to_quaternion(cube_pose, [0, 0, 0, 0, 0, 0])
        # self.ur5_arm.set_target_pose(cmd, t=self.agent_control_dt)
        # self.rate.sleep()
        # gripper_cmd = self.scale_gripper(1.)
        # self.ur5_gripper.gripper_joint_control(gripper_cmd)

        self.rate.sleep()

    
    def _is_success(self, achieved_goal, desired_goal):
        d = goal_distance(achieved_goal, desired_goal)
        f = goal_distance(self.cube_pose, self.cube_target_pose)
        # self._log_message = "Final distance error: " + str(np.round(d, 3)) \
        #                     + (' success!' if d < self.distance_threshold else '') \
        #                     + (' left has object!' if self.left_gripper_attached else '') \
        #                     + (' right has object!' if self.right_gripper_attached else '')
        return bool(d < self.distance_threshold and f < self.distance_threshold and bool(self.left_gripper_attached or self.right_gripper_attached))