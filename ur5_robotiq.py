import numpy as np
import warnings
from collections import namedtuple
import math
import os

class UR5Robotiq85:
    def __init__(self, pb, robot_params, use_gui):
        self.vis = use_gui
        self._pb = pb
        self.arm_num = 2
        self.arm_num_dofs = 6
        self.action_scale = 0.2
        self.gripper_scale = 0.02
        self.left_tcp_link_name = 'left_robotiq_arg2f_base_link'
        self.right_tcp_link_name = 'right_robotiq_arg2f_base_link'
        self.left_arm_left_finger_pad_name = 'left_arm_left_inner_finger_pad'
        self.left_arm_right_finger_pad_name = 'left_arm_right_inner_finger_pad'
        self.right_arm_left_finger_pad_name = 'right_arm_left_inner_finger_pad'
        self.right_arm_right_finger_pad_name = 'right_arm_right_inner_finger_pad'
        
    
        self.arm_rest_poses = robot_params["reset_arm_poses"]  # default joint pose for ur5
        self.gripper_range = robot_params["reset_gripper_range"]
        self.load_urdf()
        
        
        # set info specific to arm
        self.setup_ur5_info()
        self.setup_gripper_info()

        # reset the arm to rest poses
        # self.reset()

    def close(self):
        if self._pb.isConnected():
            self._pb.disconnect()

    def load_urdf(self):
        """
        Load the robot arm model into pybullet
        """
        self.base_pos = [0, 0, 0]
        self.base_rpy = [0, 0, 0]
        self.base_orn = self._pb.getQuaternionFromEuler(self.base_rpy)
        asset_name = "./assets/urdf/dual_robot.urdf"
        self.embodiment_id = self._pb.loadURDF(asset_name, self.base_pos, self.base_orn, useFixedBase=True, flags=self._pb.URDF_ENABLE_CACHED_GRAPHICS_SHAPES)

        # create dicts for mapping link/joint names to corresponding indices
        self.num_joints, self.link_name_to_index, self.joint_name_to_index = self.create_link_joint_mappings(self.embodiment_id)

        # get the link and tcp IDs
        self.left_tcp_link_id = self.link_name_to_index[self.left_tcp_link_name]
        self.right_tcp_link_id = self.link_name_to_index[self.right_tcp_link_name]
        self.left_arm_left_finger_pad_id = self.link_name_to_index[self.left_arm_left_finger_pad_name]
        self.left_arm_right_finger_pad_id = self.link_name_to_index[self.left_arm_right_finger_pad_name]
        self.right_arm_left_finger_pad_id = self.link_name_to_index[self.right_arm_left_finger_pad_name]
        self.right_arm_right_finger_pad_id = self.link_name_to_index[self.right_arm_right_finger_pad_name]

    def create_link_joint_mappings(self, urdf_id):

        num_joints = self._pb.getNumJoints(urdf_id)

        # pull relevent info for controlling the robot
        joint_name_to_index = {}
        link_name_to_index = {}
        for i in range(num_joints):
            info = self._pb.getJointInfo(urdf_id, i)
            joint_name = info[1].decode("utf-8")
            link_name = info[12].decode("utf-8")
            joint_name_to_index[joint_name] = i
            link_name_to_index[link_name] = i

        return num_joints, link_name_to_index, joint_name_to_index
    def reset(self):
        for rest_pose, joint_id in zip(self.arm_rest_poses, self.arm_controllable_joints):
            self._pb.changeDynamics(self.embodiment_id, joint_id, linearDamping=0.04, angularDamping=0.04)
            self._pb.changeDynamics(self.embodiment_id, joint_id, jointDamping=0.01)
            self._pb.resetJointState(self.embodiment_id, joint_id, rest_pose, 0)
        # self.step_simulation()
        open_angle = 0.715 - math.asin((self.gripper_range[1] - 0.010) / 0.1143)  # angle calculation
        self._pb.setJointMotorControl2(self.embodiment_id, self.left_mimic_parent_id, self._pb.POSITION_CONTROL, targetPosition=open_angle,
                                force=self.joints[self.left_mimic_parent_id].maxForce, maxVelocity=self.joints[self.left_mimic_parent_id].maxVelocity)
        self._pb.setJointMotorControl2(self.embodiment_id, self.right_mimic_parent_id, self._pb.POSITION_CONTROL, targetPosition=open_angle,
                            force=self.joints[self.right_mimic_parent_id].maxForce, maxVelocity=self.joints[self.right_mimic_parent_id].maxVelocity)
        

    def move_gripper(self, action):
        left_current_gripper_open_length = math.sin(0.715-self._pb.getJointState(self.embodiment_id, self.left_mimic_parent_id)[0])*0.1143 + 0.010
        right_current_gripper_open_length = math.sin(0.715-self._pb.getJointState(self.embodiment_id, self.right_mimic_parent_id)[0])*0.1143 + 0.010
        left_target_gripper_open_length = np.clip(left_current_gripper_open_length + action[0] * self.gripper_scale, self.gripper_range[0], self.gripper_range[1])
        right_target_gripper_open_length = np.clip(right_current_gripper_open_length + action[-1] * self.gripper_scale, self.gripper_range[0], self.gripper_range[1])
        left_open_angle = 0.715 - math.asin((left_target_gripper_open_length - 0.010) / 0.1143)  # angle calculation
        right_open_angle = 0.715 - math.asin((right_target_gripper_open_length - 0.010) / 0.1143)  # angle calculation
        # Control the mimic gripper joint(s)
        self._pb.setJointMotorControl2(self.embodiment_id, self.right_mimic_parent_id, self._pb.POSITION_CONTROL, targetPosition=right_open_angle,
                                force=self.joints[self.right_mimic_parent_id].maxForce, maxVelocity=self.joints[self.right_mimic_parent_id].maxVelocity)
        self._pb.setJointMotorControl2(self.embodiment_id, self.left_mimic_parent_id, self._pb.POSITION_CONTROL, targetPosition=left_open_angle,
                                force=self.joints[self.left_mimic_parent_id].maxForce, maxVelocity=self.joints[self.left_mimic_parent_id].maxVelocity)
        # pos, _, _, _ = self._pb.getJointState(self.embodiment_id, self.right_mimic_parent_id)
        # print(pos)
        
    def move_ee(self, action, control_method):
        '''
        Move the end effector of the robot
        action: (np.ndarray)
        '''
        assert control_method in ('joint', 'end')
        if control_method == 'end':
            ee_displacement = action * self.action_scale  # limit maximum change in position
            left_ee_position =np.array(self._pb.getLinkState(self.embodiment_id, self.left_tcp_link_id)[4])
            right_ee_position =np.array(self._pb.getLinkState(self.embodiment_id, self.right_tcp_link_id)[4])
            # self._pb.addUserDebugPoints(pointPositions = [left_ee_position], pointColorsRGB = [[0, 0, 255]], pointSize= 40, lifeTime= 0)
            left_target_ee_position = left_ee_position + ee_displacement[:3]
            right_target_ee_position = right_ee_position + ee_displacement[3:]
            # Clip the height target. For some reason, it has a great impact on learning
            left_target_ee_position[2] = np.max((0, left_target_ee_position[2]))
            right_target_ee_position[2] = np.max((0, right_target_ee_position[2]))
            left_arm_joint_poses =np.array(self._pb.calculateInverseKinematics(self.embodiment_id, self.left_tcp_link_id, 
                                                                      left_target_ee_position,
                                                                    #   targetOrientation = self._pb.getQuaternionFromEuler([0, math.pi/2, 0]),
                                                                      lowerLimits = self.arm_lower_limits[:6], 
                                                                      upperLimits = self.arm_upper_limits[:6], 
                                                                      jointRanges = self.arm_joint_ranges[:6],
                                                                    #   solver = 1,
                                                                      maxNumIterations=200))
            right_arm_joint_poses =np.array(self._pb.calculateInverseKinematics(self.embodiment_id, self.right_tcp_link_id, 
                                                                      right_target_ee_position,
                                                                    #   targetOrientation = self._pb.getQuaternionFromEuler([0, math.pi/2, 0]),
                                                                      lowerLimits = self.arm_lower_limits[6:], 
                                                                      upperLimits = self.arm_upper_limits[6:], 
                                                                      jointRanges = self.arm_joint_ranges[6:],
                                                                    #   solver = 1,
                                                                      maxNumIterations=200))
            left_joint_poses = left_arm_joint_poses[:self.arm_num_dofs]
            right_joint_poses = right_arm_joint_poses[12:12+self.arm_num_dofs]
            joint_poses = np.concatenate([left_joint_poses, right_joint_poses])
            # self._pb.addUserDebugPoints(pointPositions = [target_ee_position], pointColorsRGB = [[0, 0, 255]], pointSize= 40, lifeTime= 0)
        elif control_method == 'joint':
            assert len(action) == self.arm_num_dofs
            arm_joint_ctrl = action * self.action_scale  # limit maximum change in position
            current_arm_joint_angles = np.array([self._pb.getJointState(self.embodiment_id, i)[0] for i in self.arm_controllable_joints])
            joint_poses = current_arm_joint_angles + arm_joint_ctrl
        # arm
        for i, joint_id in enumerate(self.arm_controllable_joints):
            self._pb.setJointMotorControl2(self.embodiment_id, joint_id, self._pb.POSITION_CONTROL, joint_poses[i],
                                    force=self.joints[joint_id].maxForce, maxVelocity=self.joints[joint_id].maxVelocity)

    def setup_ur5_info(self):
        """
        Set some of the parameters used when controlling the UR5
        """

        jointInfo = namedtuple('jointInfo', 
            ['id','name','type','damping','friction','lowerLimit','upperLimit','maxForce','maxVelocity','controllable','linkname'])
        self.joints = []
        self.control_joint_ids = []
        for i in range(self._pb.getNumJoints(self.embodiment_id)):
            info = self._pb.getJointInfo(self.embodiment_id, i)
            jointID = info[0]
            jointName = info[1].decode("utf-8")
            jointType = info[2]  # JOINT_REVOLUTE, JOINT_PRISMATIC, JOINT_SPHERICAL, JOINT_PLANAR, JOINT_FIXED
            jointDamping = info[6]
            jointFriction = info[7]
            jointLowerLimit = info[8]
            jointUpperLimit = info[9]
            jointMaxForce = info[10]
            jointMaxVelocity = info[11]
            linkName = info[12].decode("utf-8")
            controllable = (jointType != self._pb.JOINT_FIXED)
            if controllable:
                self.control_joint_ids.append(jointID)
                self._pb.setJointMotorControl2(self.embodiment_id, jointID, self._pb.VELOCITY_CONTROL, targetVelocity=0, force=0)
            info = jointInfo(jointID,jointName,jointType,jointDamping,jointFriction,jointLowerLimit,
                            jointUpperLimit,jointMaxForce,jointMaxVelocity,controllable,linkName)
            self.joints.append(info)
        self.control_joint_names = [self.joints[i].name for i in self.control_joint_ids]

        # get the control and calculate joint ids in list form, useful for pb array methods
        assert self.control_joint_ids == [self.joint_name_to_index[name] for name in self.control_joint_names]
        self.num_control_dofs = len(self.control_joint_ids)
        assert self.num_control_dofs >= self.arm_num_dofs*self.arm_num
        self.arm_controllable_joints = self.control_joint_ids[:self.arm_num_dofs]+self.control_joint_ids[self.arm_num_dofs+6:self.arm_num_dofs+12]
        arm_lower_limits_control = [info.lowerLimit for info in self.joints if info.controllable]
        self.arm_lower_limits = arm_lower_limits_control[:self.arm_num_dofs]+arm_lower_limits_control[self.arm_num_dofs+6:self.arm_num_dofs+12]
        arm_upper_limits_control = [info.upperLimit for info in self.joints if info.controllable]
        self.arm_upper_limits = arm_upper_limits_control[:self.arm_num_dofs]+arm_upper_limits_control[self.arm_num_dofs+6:self.arm_num_dofs+12]
        arm_joint_ranges_control = [info.upperLimit - info.lowerLimit for info in self.joints if info.controllable]
        self.arm_joint_ranges = arm_joint_ranges_control[:self.arm_num_dofs]+arm_joint_ranges_control[self.arm_num_dofs+6:self.arm_num_dofs+12]
    
    def setup_gripper_info(self):
        self.left_mimic_parent_id = [joint.id for joint in self.joints if joint.name == 'left_finger_joint'][0]
        self.right_mimic_parent_id = [joint.id for joint in self.joints if joint.name == 'right_finger_joint'][0]
        self.__setup_mimic_joints__('right')
        self.__setup_mimic_joints__('left')
        
        
    def __setup_mimic_joints__(self, gripper_direction):
        erp = 1
        if gripper_direction == 'left':
            

            right_outer_knuckle_id = [joint.id for joint in self.joints if joint.linkname == 'left_arm_right_outer_knuckle'][0]
            c = self._pb.createConstraint(self.embodiment_id, self.left_mimic_parent_id,
                                    self.embodiment_id, right_outer_knuckle_id,
                                    jointType=self._pb.JOINT_GEAR,
                                    jointAxis=[1, 0, 0],
                                    parentFramePosition=[0, 0, 0],
                                    childFramePosition=[0, 0, 0])
            self._pb.changeConstraint(c, gearRatio=-1, erp=erp)

            left_inner_finger_id = [joint.id for joint in self.joints if joint.linkname == 'left_arm_left_inner_finger'][0]
            c = self._pb.createConstraint(self.embodiment_id, self.left_mimic_parent_id,
                                    self.embodiment_id, left_inner_finger_id,
                                    jointType=self._pb.JOINT_GEAR,
                                    jointAxis=[1, 0, 0],
                                    parentFramePosition=[0, 0, 0],
                                    childFramePosition=[0, 0, 0])
            self._pb.changeConstraint(c, gearRatio=1, erp=erp)

            left_inner_knuckle_id = [joint.id for joint in self.joints if joint.linkname == 'left_arm_left_inner_knuckle'][0]
            c = self._pb.createConstraint(self.embodiment_id, self.left_mimic_parent_id,
                                    self.embodiment_id, left_inner_knuckle_id,
                                    jointType=self._pb.JOINT_GEAR,
                                    jointAxis=[1, 0, 0],
                                    parentFramePosition=[0, 0, 0],
                                    childFramePosition=[0, 0, 0])
            self._pb.changeConstraint(c, gearRatio=-1, erp=erp)


            right_inner_finger_id = [joint.id for joint in self.joints if joint.linkname == 'left_arm_right_inner_finger'][0]
            c = self._pb.createConstraint(self.embodiment_id, right_outer_knuckle_id,
                                    self.embodiment_id, right_inner_finger_id,
                                    jointType=self._pb.JOINT_GEAR,
                                    jointAxis=[1, 0, 0],
                                    parentFramePosition=[0, 0, 0],
                                    childFramePosition=[0, 0, 0])
            self._pb.changeConstraint(c, gearRatio=1, erp=erp)

            right_inner_knuckle_id = [joint.id for joint in self.joints if joint.linkname == 'left_arm_right_inner_knuckle'][0]
            c = self._pb.createConstraint(self.embodiment_id, right_outer_knuckle_id,
                                    self.embodiment_id, right_inner_knuckle_id,
                                    jointType=self._pb.JOINT_GEAR,
                                    jointAxis=[1, 0, 0],
                                    parentFramePosition=[0, 0, 0],
                                    childFramePosition=[0, 0, 0])
            self._pb.changeConstraint(c, gearRatio=-1, erp=erp)

        elif gripper_direction == 'right':
            

            right_outer_knuckle_id = [joint.id for joint in self.joints if joint.linkname == 'right_arm_right_outer_knuckle'][0]
            c = self._pb.createConstraint(self.embodiment_id, self.right_mimic_parent_id,
                                    self.embodiment_id, right_outer_knuckle_id,
                                    jointType=self._pb.JOINT_GEAR,
                                    jointAxis=[1, 0, 0],
                                    parentFramePosition=[0, 0, 0],
                                    childFramePosition=[0, 0, 0])
            self._pb.changeConstraint(c, gearRatio=-1, erp=erp)

            left_inner_finger_id = [joint.id for joint in self.joints if joint.linkname == 'right_arm_left_inner_finger'][0]
            c = self._pb.createConstraint(self.embodiment_id, self.right_mimic_parent_id,
                                    self.embodiment_id, left_inner_finger_id,
                                    jointType=self._pb.JOINT_GEAR,
                                    jointAxis=[1, 0, 0],
                                    parentFramePosition=[0, 0, 0],
                                    childFramePosition=[0, 0, 0])
            self._pb.changeConstraint(c, gearRatio= 1, erp=erp)

            left_inner_knuckle_id = [joint.id for joint in self.joints if joint.linkname == 'right_arm_left_inner_knuckle'][0]
            c = self._pb.createConstraint(self.embodiment_id, self.right_mimic_parent_id,
                                    self.embodiment_id, left_inner_knuckle_id,
                                    jointType=self._pb.JOINT_GEAR,
                                    jointAxis=[1, 0, 0],
                                    parentFramePosition=[0, 0, 0],
                                    childFramePosition=[0, 0, 0])
            self._pb.changeConstraint(c, gearRatio=-1, erp=erp)


            right_inner_finger_id = [joint.id for joint in self.joints if joint.linkname == 'right_arm_right_inner_finger'][0]
            c = self._pb.createConstraint(self.embodiment_id, right_outer_knuckle_id,
                                    self.embodiment_id, right_inner_finger_id,
                                    jointType=self._pb.JOINT_GEAR,
                                    jointAxis=[1, 0, 0],
                                    parentFramePosition=[0, 0, 0],
                                    childFramePosition=[0, 0, 0])
            self._pb.changeConstraint(c, gearRatio=1, erp=erp)

            right_inner_knuckle_id = [joint.id for joint in self.joints if joint.linkname == 'right_arm_right_inner_knuckle'][0]
            c = self._pb.createConstraint(self.embodiment_id, right_outer_knuckle_id,
                                    self.embodiment_id, right_inner_knuckle_id,
                                    jointType=self._pb.JOINT_GEAR,
                                    jointAxis=[1, 0, 0],
                                    parentFramePosition=[0, 0, 0],
                                    childFramePosition=[0, 0, 0])
            self._pb.changeConstraint(c, gearRatio=-1, erp=erp)



    def step_simulation(self):
        raise RuntimeError('`step_simulation` method of RobotBase Class should be hooked by the environment.')
    
    def get_joint_obs(self,control_type) -> np.ndarray:
        positions = []
        if control_type == 'joint':
            control_joint_ls = self.control_joint_ids
            for joint_id in control_joint_ls:
                pos, _, _, _ = self._pb.getJointState(self.embodiment_id, joint_id)
                positions.append(pos)
        else:
            # positions_arm = self._pb.getLinkState(self.embodiment_id, self.tcp_link_id)[4]
            left_arm_left_finger_info = self._pb.getLinkState(self.embodiment_id, self.left_arm_left_finger_pad_id,computeLinkVelocity=1)
            left_arm_right_finger_info = self._pb.getLinkState(self.embodiment_id, self.left_arm_right_finger_pad_id,computeLinkVelocity=1)
            left_arm_finger_pos = list((np.array(left_arm_left_finger_info[4])+np.array(left_arm_right_finger_info[4]))/2)
            right_arm_left_finger_info = self._pb.getLinkState(self.embodiment_id, self.right_arm_left_finger_pad_id,computeLinkVelocity=1)
            right_arm_right_finger_info = self._pb.getLinkState(self.embodiment_id, self.right_arm_right_finger_pad_id,computeLinkVelocity=1)
            right_arm_finger_pos = list((np.array(right_arm_left_finger_info[4])+np.array(right_arm_right_finger_info[4]))/2)
            left_ee_orn = list(self._pb.getEulerFromQuaternion(np.array(self._pb.getLinkState(self.embodiment_id, self.left_tcp_link_id)[5])))
            right_ee_orn = list(self._pb.getEulerFromQuaternion(np.array(self._pb.getLinkState(self.embodiment_id, self.right_tcp_link_id)[5])))
            left_arm_finger_linear_veocity = list((np.array(left_arm_left_finger_info[6])+np.array(left_arm_right_finger_info[6]))/2)
            right_arm_finger_linear_veocity = list((np.array(right_arm_left_finger_info[6])+np.array(right_arm_right_finger_info[6]))/2)
            left_arm_finger_angular_veocity = list((np.array(left_arm_left_finger_info[7])+np.array(left_arm_right_finger_info[7]))/2)
            right_arm_finger_angular_veocity = list((np.array(right_arm_left_finger_info[7])+np.array(right_arm_right_finger_info[7]))/2)
            arm_obs = left_arm_finger_pos+right_arm_finger_pos+left_ee_orn+right_ee_orn+left_arm_finger_linear_veocity+right_arm_finger_linear_veocity+left_arm_finger_angular_veocity+right_arm_finger_angular_veocity
            # self._pb.addUserDebugPoints(pointPositions = [(np.array(right_arm_left_finger_info[4])+np.array(right_arm_right_finger_info[4]))/2], pointColorsRGB = [[0, 0, 255]], pointSize= 40, lifeTime= 0)

            left_positions_gripper = []
            right_positions_gripper = []
            for joint_id in self.control_joint_ids[self.arm_num_dofs:self.arm_num_dofs+6]:
                pos, _, _, _ = self._pb.getJointState(self.embodiment_id, joint_id)
                left_positions_gripper.append(pos)
            for joint_id in self.control_joint_ids[self.arm_num_dofs+12:]:
                pos, _, _, _ = self._pb.getJointState(self.embodiment_id, joint_id)
                right_positions_gripper.append(pos)
            positions = arm_obs+left_positions_gripper+right_positions_gripper

        robot_obs = np.array(positions)
        return robot_obs