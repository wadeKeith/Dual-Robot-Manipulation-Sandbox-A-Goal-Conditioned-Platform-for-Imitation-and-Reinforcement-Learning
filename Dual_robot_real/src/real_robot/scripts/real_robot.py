import rospy
from trajectory_msgs.msg import *
from control_msgs.msg import *
from utils.real_arm import Real_arm
import numpy as np
import utils.transformations as transformations
from utils.utils import PID
# import PyKDL

def goal_distance(goal_a, goal_b):
        return np.linalg.norm(goal_a - goal_b, axis=-1)
class Real_robot(object):

    def __init__(self):
        self.left_arm = Real_arm('/left/')
        self.right_arm = Real_arm('/right/')
        # self.left_arm.arm_init()
        
        self.left_init_q = [0, -1.744, -1.396, -1.57, -2.355, 0] #[0, -2.355, -0.785, -1.57, -2.355, 0]
        self.right_init_q = [0, -1.396, 1.396, -1.57, 2.355, 0] #[0, -0.785, 0.785, -1.57, 2.355, 0]
        self.action_factor = 0.005
        self.action_delta_t = 0.1
        self.workspace_low = [0,-0.85,0.8]
        self.workspace_high = [0.8, 0.85, 1.6]


    def reset(self):
        # left arm reset
        self.left_arm.play_service_client()
        rospy.sleep(0.5)
        self.left_arm.set_joint_positions(self.left_init_q,wait=True,t=10)
        # self.left_arm.stop_service_client()
        # rospy.sleep(0.5)
        # right arm reset
        self.right_arm.play_service_client()
        rospy.sleep(0.5)
        self.right_arm.set_joint_positions(self.right_init_q,wait=True,t=10)
        # self.right_arm.stop_service_client()
        # rospy.sleep(0.5)
        left_pose = self.left_arm.end_effector(joint_angles=None, tip_link=self.left_arm.ee_link)
        right_pose = self.right_arm.end_effector(joint_angles=None, tip_link=self.right_arm.ee_link)
        obs = np.concatenate((left_pose[:3],transformations.euler_from_quaternion(left_pose[3:], axes='rxyz'),
                              right_pose[:3],transformations.euler_from_quaternion(right_pose[3:], axes='rxyz')))
        return obs

    def step(self, action,action_type):
        if action_type == 'joint':
            delta = action*self.action_factor
            left_cmd = self.left_arm.joint_angles()+delta[:6]
            right_cmd = self.right_arm.joint_angles() + delta[6:]

            self.left_arm.set_joint_positions_flex(left_cmd,t=self.action_delta_t)
            self.right_arm.set_joint_positions_flex(right_cmd, t = self.action_delta_t)
            obs = np.concatenate((self.left_arm.joint_angles(),self.right_arm.joint_angles()))
        elif action_type == 'xyz':
            left_cpose = self.left_arm.end_effector(joint_angles=None, tip_link=self.left_arm.ee_link)
            right_cpose = self.right_arm.end_effector(joint_angles=None, tip_link=self.right_arm.ee_link)
            delta = action* self.action_factor
            # left_delta = np.concatenate((delta[:3], [0, 0, 0])) # Do not change ax, ay, and az
            # right_delta = np.concatenate((delta[3:], [0, 0, 0])) # Do not change ax, ay, and az
            left_delta = delta[:6] # Do not change ax, ay, and az
            right_delta = delta[6:] # Do not change ax, ay, and az
            left_cmd = transformations.pose_euler_to_quaternion(pose=left_cpose, 
                                                                delta=left_delta,
                                                                workspace_low=self.workspace_low, 
                                                                workspace_high=self.workspace_high) #找一下工作空间大小
            right_cmd = transformations.pose_euler_to_quaternion(pose=right_cpose, 
                                                                 delta=right_delta, 
                                                                 workspace_low=self.workspace_low,
                                                                 workspace_high=self.workspace_high)
            self.left_arm.set_target_pose_flex(left_cmd, t=self.action_delta_t)
            self.right_arm.set_target_pose_flex(right_cmd, t = self.action_delta_t)
            left_pose_next = self.left_arm.end_effector(joint_angles=None, tip_link=self.left_arm.ee_link)
            right_pose_next = self.right_arm.end_effector(joint_angles=None, tip_link=self.right_arm.ee_link)
            obs = np.concatenate((left_pose_next[:3],transformations.euler_from_quaternion(left_pose_next[3:], axes='rxyz'),
                                  right_pose_next[:3],transformations.euler_from_quaternion(right_pose_next[3:], axes='rxyz')))
        
        return obs




if __name__ == "__main__":
    rospy.init_node('dual_robot',
                anonymous=True)
    robot = Real_robot()
    way_points = np.array([[0.569, 0.03, 1.117, 0,0,0,
                            0.186, -0.678,  0.835, -2.223, 1.57,  2.217],
                            [0.65, 0.03, 1.117, 0,0,0,
                            0.186, -0.678,  0.835, -2.223, 1.57,  2.217],
                            [0.569, 0.03, 1.117, 0,0,0,
                            0.186, -0.678,  0.835, -2.223, 1.57,  2.217],
                            [0.185,  0.683,  0.838 , -1.891,  1.567, 1.891,
                             0.186, -0.678,  0.835, -2.223, 1.57,  2.217],
                            [0.185,  0.683,  0.838 , -1.891,  1.567, 1.891,
                            0.569, 0.03, 1.2, 0,0,0],
                            [0.185,  0.683,  0.838 , -1.891,  1.567, 1.891,
                            0.186, -0.678,  0.835, -2.223, 1.57,  2.217]])
    # desired_goal = np.array([-0.5, -2.355, -0.785, -1.57, -2.355, 0, 0.5, -0.785, 0.785, -1.57, 2.355, 0])
    # init xyz pos left:[0.185,  0.683,  0.838 , -1.891,  1.567, 1.891]    right: [0.186, -0.678,  0.835, -2.223, 1.57,  2.217]
    # desired_goal = np.array()
    K_p = 0.5*np.ones(12)
    K_i = 0.01*np.ones(12)
    K_d = 0.1*np.ones(12)
    pid = PID(Kp = K_p, Ki=K_i, Kd=K_d, dynamic_pid=True, max_gain_multiplier=5)

    obs = robot.reset()
    # print(obs)
    # left_pose = robot.left_arm.end_effector(joint_angles=None,tip_link=robot.left_arm.ee_link)
    # left_world_pose = robot.left_arm.end_effector(joint_angles=None,tip_link=robot.left_arm.base_link)
    # right_pose = robot.right_arm.end_effector(joint_angles=None,tip_link=robot.right_arm.ee_link)
    # right_world_pose = robot.right_arm.end_effector(joint_angles=None,tip_link=robot.right_arm.base_link)
    # print('left_pose:',left_pose)
    # print('left_world:',left_world_pose)
    # print('right_pose:',right_pose)
    # print('right_world:',right_world_pose)
    # for i in range(len(way_points)):
    #     desired_goal = way_points[i]
    #     done = False
    #     # error  = desired_goal-obs
    #     # error_next = desired_goal-obs
    #     while not done:
    #         # error = desired_goal-obs
    #         # action = K_p*error_next + (error_next-error)/robot.action_delta_t*K_i
    #         action = pid.update(error = desired_goal-obs)
    #         # error = error_next
    #         obs = robot.step(action,'xyz')
    #         # error_next = desired_goal-obs
    #         done = bool(goal_distance(desired_goal,obs)<0.05)
    robot.left_arm.stop_service_client()
    rospy.sleep(0.5)
    robot.right_arm.stop_service_client()
    rospy.sleep(0.5)