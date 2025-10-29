#! /usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import actionlib
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from move_demo.srv import SetJointAngles, SetJointAnglesResponse 

# 定义关节名称和其他参数
JOINT_NAMES = ['elbow_joint', 'shoulder_lift_joint', 'shoulder_pan_joint', 'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']
DURATION = 0.01
INIT = [0, 0, 0, 0, 0, 0]

class MasterJointMove():
    def __init__(self, init_joints=INIT, duration=DURATION):

        self.client = actionlib.SimpleActionClient('/right/arm_controller/follow_joint_trajectory',
                                                   FollowJointTrajectoryAction)
        self.client.wait_for_server()

        self.initial = FollowJointTrajectoryGoal()
        self.initial.trajectory = JointTrajectory()
        self.initial.trajectory.joint_names = JOINT_NAMES
        self.current_joints = init_joints
        self.initial.trajectory.points = [JointTrajectoryPoint(positions=INIT, velocities=[0]*6,
                                                               time_from_start=rospy.Duration(duration))]
        self.duration = duration

    def move_to_goal(self, goal_pose):
        goal = FollowJointTrajectoryGoal()
        goal.trajectory = JointTrajectory()
        goal.trajectory.joint_names = JOINT_NAMES
        goal.trajectory.points = [JointTrajectoryPoint(positions=goal_pose, velocities=[0]*6,
                                                       time_from_start=rospy.Duration(self.duration))]
        
        # 打印目标信息
        print(goal)

        # 发送目标
        self.client.send_goal(goal)
        self.client.wait_for_result()
        return self.client.get_result() is not None

def handle_set_joint_angles(req):
    masterjointmove = MasterJointMove()
    goal_pose = [
        req.elbow_joint,
        req.shoulder_lift_joint,
        req.shoulder_pan_joint,
        req.wrist_1_joint,
        req.wrist_2_joint,
        req.wrist_3_joint
    ]
    success = masterjointmove.move_to_goal(goal_pose)
    return SetJointAnglesResponse(success=success)

def set_joint_angles_server():
    rospy.init_node('set_joint_angles_server')
    s = rospy.Service('right_joint_move', SetJointAngles, handle_set_joint_angles)
    print("Ready to set joint angles.")
    rospy.spin()

if __name__ == '__main__':
    set_joint_angles_server()
