# The MIT License (MIT)
#
# Copyright (c) 2018-2022 Cristian C Beltran-Hernandez
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.
#
# Author: Cristian C Beltran-Hernandez
import sys

import rospy
import numpy as np
from numpy.random import RandomState

from controller_py.arm import Arm
from controller_py.gripper import Gripper
import robot_gazebo_env_goal

class UR5Env(robot_gazebo_env_goal.RobotGazeboEnv):
    """Superclass for all Robot environments.
    """

    def __init__(self, driver):
        """Initializes a new Robot environment.
        """

        rospy.logwarn("Start UREnv Init")
        # Variables that we give through the constructor.

        # Internal Vars
        self.controllers_list = ['joint_state_controller','arm_controller', 'gripper']

        # It doesn't use namespace
        # self.robot_name_space = ""

        reset_controls_bool = False
        
        self.use_gazebo = False
        if driver == 'gazebo':
            self.use_gazebo = True

        # We launch the init function of the Parent Class robot_env.RobotGazeboEnv
        super(UR5Env, self).__init__(controllers_list=self.controllers_list,
                                     reset_controls=reset_controls_bool,
                                     use_gazebo=self.use_gazebo)

        self.left_arm = Arm(namespace = '/left/')
        self.right_arm = Arm(namespace = '/right/')
        self.left_gripper = Gripper(prefix = '/left/')
        self.right_gripper = Gripper(prefix = '/right/')

        if self.rand_seed is not None:
            self.seed(self.rand_seed)
            RandomState(self.rand_seed)
            np.random.seed(self.rand_seed)

        rospy.logwarn("Finished UREnv INIT...")


    # Methods needed by the RobotGazeboEnv
    # ----------------------------

    def _pause_env(self):
        current_pose = self.ur5_arm.joint_angles
        input("Press Enter to continue")
        self.ur5_arm.set_joint_positions(current_pose, wait=True, t=self.reset_time)


    def _check_all_systems_ready(self):
        """
        Checks that all the sensors, publishers and other simulation systems are
        operational.
        """
        # TODO
        return True

    # Methods that the TrainingEnvironment will need to define here as virtual
    # because they will be used in RobotGazeboEnv GrandParentClass and defined in the
    # TrainingEnvironment.
    # ----------------------------

    def _init_env_variables(self):
        """Inits variables needed to be initialized each time we reset at the start
        of an episode.
        """
        raise NotImplementedError()

    def compute_reward(self, observations, done):
        """Calculates the reward to give based on the observations given.
        """
        raise NotImplementedError()

    def _set_action(self, action):
        """Applies the given action to the simulation.
        """
        raise NotImplementedError()

    def _get_obs(self):
        raise NotImplementedError()

    def _is_success(self, achieved_goal, desired_goal):
        """Checks if episode done based on observations given.
        """
        raise NotImplementedError()
