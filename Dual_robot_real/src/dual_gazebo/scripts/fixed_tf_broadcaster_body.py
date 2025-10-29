#!/usr/bin/env python3

import rospy
import tf #改成tf2

if __name__ == '__main__':
    rospy.init_node('broadcaster_fixed_body')
    br = tf.TransformBroadcaster()
    rate = rospy.Rate(10.0)
    while not rospy.is_shutdown():
        br.sendTransform((0.0, 0.0, 0.0),
                         (0.0, 0.0, 0.0, 1.0),
                         rospy.Time.now(),
                         "body_tf/world",
                         "world")
        rate.sleep()