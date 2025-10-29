#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from std_msgs.msg import String
from sensor_msgs.msg import Image, CameraInfo
import cv2
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import Vector3
from geometry_msgs.msg import Quaternion
import tf
import numpy as np
import time


class Camera(object):
    def __init__(self,prefix):
        self.prefix = prefix
        self.image_sub = rospy.Subscriber("/%s_camera/color/image_raw" % self.prefix, Image, self.image_cb)
        self.depth_sub = rospy.Subscriber("/%s_camera/aligned_depth_to_color/image_raw" % self.prefix, Image, self.depth_cb)
        self.bridge = CvBridge()
        self.color_image = None
        self.depth_image = None

    def image_cb(self,data):
        
        self.color_image = self.bridge.imgmsg_to_cv2(data, 'bgr8')

        # cv2.imshow('color_image',self.color_image)
        # cv2.waitKey(1)
    def depth_cb(self, data):
        self.depth_image = self.bridge.imgmsg_to_cv2(data, '8UC1')
    def reset(self):
        rospy.sleep(0.5)
    def _get_rgbd_image(self):
        return self.color_image, self.depth_image


if __name__ == '__main__':
    rospy.init_node('cam-test',
                anonymous=True)
    cam = Camera(prefix='top')
    cam.reset()
    while not rospy.is_shutdown():
        # rospy.spin()
        color_img,depth_img = cam._get_rgbd_image()
    print('a')


# import pyrealsense2 as rs

# ctx = rs.context()
# if len(ctx.devices) > 0:
#     for d in ctx.devices:
#         print('Found device: ',
#               d.get_info(rs.camera_info.name), ' ',
#               d.get_info(rs.camera_info.serial_number))
# else:
#     print("No Intel Device connected")

# 接了两个D435摄像头，显示结果：
# Found device:  Intel RealSense D435   827312071726
# Found device:  Intel RealSense D435   838212074152



















