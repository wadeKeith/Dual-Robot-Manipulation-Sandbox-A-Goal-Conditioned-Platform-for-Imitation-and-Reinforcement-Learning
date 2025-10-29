import numpy as np
import pyrealsense2 as rs
from matplotlib import pyplot as plt
import cv2
import open3d as o3d
import sys
# sys.path.append('/media/jt/Extreme SSD/Github/Dual_robot_real/src/real_robot/scripts/')
from realsense_depth import DepthCamera
from utils import createPointCloudO3D
from utils import depth2PointCloud
from utils import create_point_cloud_file2
from utils import write_point_cloud

resolution_width, resolution_height = (640, 480)

clip_distance_max = 1.5 #remove from the depth image all values above a given value (meters).
                          # Disable by giving negative value (default)

def main():

    realsense_ctx = rs.context()
    
    if len(realsense_ctx.devices) > 0:
        connected_devices = []
        for camera in realsense_ctx.devices:
            connected_devices.append(camera.get_info(rs.camera_info.serial_number))
    else:
        connected_devices = None
        print("No Intel Device connected")
        
    print('Top Camera:')
    Realsensed435Cam_Top = DepthCamera(resolution_width, resolution_height, device_id=connected_devices[0])
    print('Right Camera:')
    Realsensed435Cam_Right = DepthCamera(resolution_width, resolution_height, device_id=connected_devices[1])

    depth_scale_top = Realsensed435Cam_Top.get_depth_scale()

    while True:

        ret , depth_raw_frame, color_raw_frame = Realsensed435Cam_Top.get_raw_frame()
        if not ret:
            print("Unable to get a frame")
        
        #o3D library for construct point clouds with rgbd image and camera matrix
        pcd = createPointCloudO3D(color_raw_frame, depth_raw_frame, depth_scale_top,  clip_distance_max)
        #o3d.visualization.draw_geometries([pcd]) 
        
        #numpy with point cloud generation
        points_xyz_rgb = depth2PointCloud(depth_raw_frame, color_raw_frame, depth_scale_top, clip_distance_max)
        #write_point_cloud("chair2.ply", points_xyz_rgb)
        add_pc = np.array([[0,0,0,0,0,0],
                  [0.1,0,0,0,0,255],
                  [0,0.1,0,0,255,0],
                  [0,0,0.1,255,0,0]])
        points_xyz_rgb = np.concatenate((points_xyz_rgb, add_pc), axis=0)
        create_point_cloud_file2(points_xyz_rgb,"door_create.ply")

        # create_point_cloud_file2(np.concatenate((np.asanyarray(pcd.points), np.asanyarray(pcd.colors)),axis=-1), "door_o3d_file.ply")


                
        #plt.show()
        color_frame = np.asanyarray(color_raw_frame.get_data())
        depth_frame = np.asanyarray(depth_raw_frame.get_data())
        print("frame shape:", color_frame.shape)
        cv2.imshow("Frame",  color_frame )
        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            cv2.imwrite("frame_color.png", color_frame)
            plt.imsave("frame_depth.png", depth_frame)
            break
    
    Realsensed435Cam_Top.release() # release rs pipeline
    Realsensed435Cam_Right.release()


if __name__ == '__main__':
    main()
