#!/usr/bin/env python3
# -*- coding: utf-8 -*-
from trajectory_msgs.msg import *
from control_msgs.msg import *
from move_demo.srv import GripperControl, GripperControlResponse  # Import the service
import actionlib
import rospy

FINGER_JOINT_NAMES = ['gripper_finger1_joint']
 
def commandCallback(req):
    gripper_goal = FollowJointTrajectoryGoal()

    gripper_goal.trajectory = JointTrajectory()
    gripper_goal.trajectory.joint_names =  ['gripper_finger1_joint']
    gripper_goal.trajectory.points.append(JointTrajectoryPoint())

    gripper_goal.trajectory.points[0].positions = [req.width]
    gripper_goal.trajectory.points[0].time_from_start = rospy.Duration(1)
    gripper_client.send_goal(gripper_goal)
    gripper_client.wait_for_result()
    return GripperControlResponse(True)

def main():
    global gripper_client
    rospy.init_node("pub_dual_arm_move")
    gripper_client = actionlib.SimpleActionClient('/left/gripper/follow_joint_trajectory', FollowJointTrajectoryAction)
    rospy.loginfo('start gripper service')
    rospy.Service('/gripper_move_left', GripperControl, commandCallback)
    rospy.spin()
 
if __name__ == "__main__":
    main()
