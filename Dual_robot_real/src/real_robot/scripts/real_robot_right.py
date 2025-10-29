import rospy
from trajectory_msgs.msg import *
from control_msgs.msg import *
from utils.real_arm import Real_arm
import numpy as np
import utils.transformations as transformations
from utils.utils import PID
from gripper_2f import Gripper2f
from vision_camera import Camera
from utils.utils import PID

def goal_distance(goal_a, goal_b):
        return np.linalg.norm(goal_a - goal_b, axis=-1)
class Robot_right(object):

    def __init__(self):
        self.right_arm = Real_arm('/right/')
        self.right_gripper = Gripper2f()
        self.top_camera = Camera(prefix='top')
        self.right_camera = Camera(prefix='right')

        self.action_type = 'djoint'
        self.right_init_q = [0, -1.396, 1.396, -1.57, 2.355, 0] #[0, -0.785, 0.785, -1.57, 2.355, 0]
        self.action_delta_t = 0.1
        self.action_factor = 1
        if self.action_type == 'djoint':
            self.arm_action_factor = np.deg2rad(170)*self.action_delta_t* self.action_factor
            # self.arm_action_factor =1
        elif self.action_type == 'joint':
            self.arm_action_factor =1
        elif self.action_type == 'xyz':
            self.arm_action_factor = 0.001 # manual setting
        self.gripper_2f_action_factor = 1*self.action_delta_t
        self.rate = rospy.Rate(1/self.action_delta_t)
        self.workspace_low = [0,-0.85,0.8]
        self.workspace_high = [0.8, 0.85, 1.6]


    def reset(self):

        # right arm reset
        self.right_arm.play_service_client()
        rospy.sleep(0.5)
        self.right_arm.set_joint_positions(self.right_init_q,wait=True,t=10)

        self.right_gripper.reset()
        obs = self._get_right_obs()
        obs_xyz = self.right_arm.end_effector(joint_angles=None, tip_link=self.right_arm.ee_link)
        top_img,top_depth = self.top_camera._get_rgbd_image()
        right_img, right_depth = self.right_camera._get_rgbd_image()
        return {'top_img':top_img,
                'top_depth':top_depth,
                'right_img':right_img,
                'right_depth':right_depth,
                'agent_pos':obs,
                'xyz_q':obs_xyz}

    def step(self, action):
        self._set_right_arm_action(action[:-1])
        self._set_right_gripper_action(action[-1])
        obs = self._get_right_obs()
        obs_xyz = self.right_arm.end_effector(joint_angles=None, tip_link=self.right_arm.ee_link)
        top_img,top_depth = self.top_camera._get_rgbd_image()
        right_img, right_depth = self.right_camera._get_rgbd_image()
        return {'top_img':top_img,
                'top_depth':top_depth,
                'right_img':right_img,
                'right_depth':right_depth,
                'agent_pos':obs,
                'xyz_q':obs_xyz}

    def _set_right_arm_action(self, action):
        if self.action_type == 'joint':
            assert len(action) == 12
            pos_cmd = action[:6]
            vel_cmd = action[6:12]
            # eff_cmd = action[12:]

            self.right_arm.set_joint_positions_flex(pos_cmd,t=self.action_delta_t, v= vel_cmd)
        elif self.action_type == 'djoint':
            assert len(action) == 6
            delta = action * self.arm_action_factor
            cmd = self.right_arm.joint_angles()+delta

            self.right_arm.set_joint_positions_flex(cmd,t=self.action_delta_t)
        elif self.action_type == 'xyz':
            assert len(action) == 3
            delta = action* self.arm_action_factor
            cpose = self.right_arm.end_effector(joint_angles=None, tip_link=self.right_arm.ee_link)

            cmd = transformations.pose_euler_to_quaternion(pose=cpose, 
                                                                 delta=delta, 
                                                                 workspace_low=self.workspace_low,
                                                                 workspace_high=self.workspace_high)

            self.right_arm.set_target_pose_flex(cmd, t = self.action_delta_t)

        self.rate.sleep()

    def _set_right_gripper_action(self,action):
        # assert len(action) == 1
        if self.action_type == 'joint':
            cmd = action
            self.right_gripper.set_joint_positions_flex(cmd,self.action_delta_t)
        else:
            delta = action * self.gripper_2f_action_factor
            cmd = self.right_gripper.get_gripper_pos() + delta
            self.right_gripper.set_joint_positions_flex(cmd,self.action_delta_t)
        self.rate.sleep()


    def _get_right_obs(self):
        if self.action_type == 'joint':
            obs = np.concatenate((self.right_arm.joint_angles(),
                                  self.right_arm.joint_velocities(),
                                #   self.right_arm.joint_efforts(),
                                  self.right_gripper.get_gripper_pos().reshape(-1,)))
        elif self.action_type == 'djoint':
            obs = np.concatenate((self.right_arm.joint_angles(),
                                  self.right_arm.joint_velocities(),
                                  self.right_gripper.get_gripper_pos().reshape(-1,)))
        elif self.action_type == 'xyz':
            right_arm_pose = self.right_arm.end_effector(joint_angles=None, tip_link=self.right_arm.ee_link)
            right_gripper_pose = self.right_gripper.get_gripper_pos()
            obs = np.concatenate((right_arm_pose[:3],transformations.euler_from_quaternion(right_arm_pose[3:], axes='rxyz'),right_gripper_pose.reshape(-1,)))
        return obs


if __name__ == "__main__":
    rospy.init_node('right_robot',
                anonymous=True)
    robot = Robot_right()
    robot.right_gripper.reset()
    robot.right_gripper.set_joint_positions_flex(1,0.1)
    rospy.sleep(2)