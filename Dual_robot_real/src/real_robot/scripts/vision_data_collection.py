import rospy
import numpy as np
from vision_camera import Camera

img_ls = []
depth_ls = []

rospy.init_node('vision_data_collection',
                anonymous=True)
cam = Camera(prefix='right')
cam.reset()
while not rospy.is_shutdown():
    # rospy.spin()
    color_img,depth_img = cam._get_rgbd_image()
    img_ls.append(color_img)
    depth_ls.append(depth_img)
    rospy.sleep(0.05)

rgbd = {'img_ls':np.array(img_ls),
        'depth_ls':np.array(depth_ls)}


np.save('class0',rgbd)