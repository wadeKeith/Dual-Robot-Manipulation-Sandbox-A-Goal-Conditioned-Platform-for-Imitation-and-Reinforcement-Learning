#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from std_msgs.msg import String
from sensor_msgs.msg import Image, CameraInfo
import message_filters
import cv2
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import Vector3
from geometry_msgs.msg import Quaternion
import tf
import numpy as np
import time
image_list = []
pose_list = []
IMAGE_PATH = '/media/jt/Extreme SSD/Github/Dual_robot_real/src/visual_realsense/output/image/'
LABEL_PATH = '/media/jt/Extreme SSD/Github/Dual_robot_real/src/visual_realsense/output/label/'


def callback(data1):
    bridge = CvBridge()
    color_image = bridge.imgmsg_to_cv2(data1, 'bgr8')
    cv2.imshow('color_image',color_image)
    cv2.waitKey(1)
    image_list.append(color_image)


def write_label_to_txt(lhw, Tcamera_o, Tcamera_object, calib, file_path, file_name):
    label = {}
    label['lhw'] = lhw
    label['Tcamera_o'] =  Tcamera_o.reshape(16)
    label['Tcamera_object'] =  Tcamera_object.reshape(16)
    label['calib'] = calib.reshape(9)
    with open(file_path +'%s.txt' % file_name, 'w') as f:
        for key, value in label.items():
            key = [key+":"]
            anno = key + value.tolist()         
            anno = [str(x) for x in anno]
            anno[-1] = anno[-1]+'\n'
            f.writelines(' '.join(anno))

if __name__ == '__main__':
    start_time = time.time()
    rospy.init_node('get_image', anonymous=True)
    try:
        
        color = message_filters.Subscriber("/camera/color/image_raw", Image)
        color.registerCallback(callback)  
        rospy.spin()
    finally:
        final_time = time.time()
        fps = len(image_list)/(final_time - start_time)
        print(fps)
        count  = 0
        for image in image_list:
            cv2.imwrite(IMAGE_PATH + '%04d.png'%count, image)
            count = count + 1



    # rate = rospy.Rate(10.0)

    # while not rospy.is_shutdown():
    #     try:
    #         # if listener.canTransform("left_wrist_1_link", "left_wrist_3_link", rospy.Time().now()):
    #         (trans,rot) = listener.lookupTransform('desk_base_link', 'left_camera_realsense_color_frame', rospy.Time(0))
    #         rospy.loginfo(trans)
    #         rospy.loginfo(rot)
    #     except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
    #         continue
    # rospy.spin()
    # rate.sleep()


    