#!/usr/bin/env python3
import math
import rospy
from tf.transformations import quaternion_from_euler
import tf



if __name__ == '__main__':
    # quaternion = quaternion_from_euler(0, 0, math.pi)

    rospy.init_node('broadcaster_fixed_right_arm')
    br = tf.TransformBroadcaster()
    rate = rospy.Rate(50.0)
    while not rospy.is_shutdown():
        br.sendTransform((0.0, 0.0, 0.0),
                         (0.0, 0.0, 0.0, 1.0),
                         rospy.Time.now(),
                         "right_arm_tf/world",
                         "world")
        rate.sleep()
