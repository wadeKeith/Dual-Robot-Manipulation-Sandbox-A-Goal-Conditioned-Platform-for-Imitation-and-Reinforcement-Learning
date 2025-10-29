#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from move_demo.srv import SetJointAngles, SetJointAnglesRequest

import time
# time.sleep(0.)


def move_to_position():
    rospy.init_node('move_to_position_client2')
    
    rospy.wait_for_service('right_joint_move')
    
    try:
        set_joint_angles = rospy.ServiceProxy('right_joint_move', SetJointAngles)
        
        request = SetJointAnglesRequest()
        request.shoulder_pan_joint = 0
        request.shoulder_lift_joint = -0.6
        request.elbow_joint = 0.6
        request.wrist_1_joint = -1.57
        request.wrist_2_joint = 2.35
        request.wrist_3_joint = 0
        
        response = set_joint_angles(request)
        
        if response.success:
            rospy.loginfo("Successfully moved to the desired position.")
        else:
            rospy.loginfo("Failed to move to the desired position.")
    
    except rospy.ServiceException as e:
        rospy.logerr("Service call failed: %s" % e)

if __name__ == '__main__':
    move_to_position()
