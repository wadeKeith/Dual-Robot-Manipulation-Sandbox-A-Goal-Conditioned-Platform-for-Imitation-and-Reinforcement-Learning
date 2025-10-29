#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from sensor_msgs.msg import JointState
from move_demo.srv import SetJointAngles, SetJointAnglesRequest

# Define joint names and other parameters
JOINT_NAMES = ['elbow_joint', 'shoulder_lift_joint', 'shoulder_pan_joint', 'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']
DELTA_ANGLE = 0.1

class JointMoverClient:
    def __init__(self):
        self.current_joint_states = [0, 0, 0, 0, 0, 0]
        rospy.init_node('joint_mover_client')
        rospy.Subscriber('/right/joint_states', JointState, self.joint_state_callback)
        self.service_client = rospy.ServiceProxy('right_joint_move', SetJointAngles)
        rospy.wait_for_service('right_joint_move')

    def joint_state_callback(self, msg):
        joint_positions = {name: pos for name, pos in zip(msg.name, msg.position)}
        self.current_joint_states = [joint_positions.get(name, 0) for name in JOINT_NAMES]

    def move_joints(self):
        rospy.sleep(1)  # Allow time for joint states to be received
        goal_angles = [angle - DELTA_ANGLE for angle in self.current_joint_states]

        req = SetJointAnglesRequest()
        req.elbow_joint = goal_angles[0]
        req.shoulder_lift_joint = goal_angles[1]
        req.shoulder_pan_joint = goal_angles[2]
        req.wrist_1_joint = goal_angles[3]
        req.wrist_2_joint = goal_angles[4]
        req.wrist_3_joint = goal_angles[5]

        # Print target information
        # rospy.loginfo(f"Requesting move to joint angles: {goal_angles}")

        try:
            response = self.service_client(req)
            if response.success:
                rospy.loginfo("Movement successful")
            else:
                rospy.logwarn("Movement failed")
        except rospy.ServiceException as e:
            print(e)
            # rospy.logerr(f"Service call failed: {e}")

if __name__ == '__main__':
    joint_mover = JointMoverClient()
    joint_mover.move_joints()
