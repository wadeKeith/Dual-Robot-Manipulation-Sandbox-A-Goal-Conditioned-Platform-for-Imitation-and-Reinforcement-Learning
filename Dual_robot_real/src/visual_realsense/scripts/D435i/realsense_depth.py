import pyrealsense2 as rs
import numpy as np

class DepthCamera:
    def __init__(self, resolution_width, resolution_height,device_id):
        # Configure depth and color streams
        self.pipeline = rs.pipeline()
        config = rs.config()
        # Create an align object
        align_to = rs.stream.color
        self.align = rs.align(align_to)
        # Set config
        config.enable_device(device_id)
        config.enable_stream(rs.stream.depth,  resolution_width,  resolution_height, rs.format.z16, 6)
        config.enable_stream(rs.stream.color,  resolution_width,  resolution_height, rs.format.bgr8, 30)
        
        # Start streaming
        self.profile = self.pipeline.start(config)
        
        self.depth_scale = self.profile.get_device().first_depth_sensor().get_depth_scale()

        print("Device product name:", self.profile.get_device().get_info(rs.camera_info.name),' Device serial number: ', device_id)
       
    def get_frame(self):
    
        # Align the depth frame to color frame
        
        frames = self.pipeline.wait_for_frames()
        aligned_frames = self.align.process(frames)
        depth_frame = aligned_frames.get_depth_frame()
        color_frame = aligned_frames.get_color_frame()

        depth_image = np.asanyarray(depth_frame.get_data())
        color_image = np.asanyarray(color_frame.get_data())
        if not depth_frame or not color_frame:
            return False, None, None
        return True, depth_image, color_image

    def get_raw_frame(self):
        frames = self.pipeline.wait_for_frames()
        aligned_frames = self.align.process(frames)
        depth_frame = aligned_frames.get_depth_frame()
        color_frame = aligned_frames.get_color_frame()
        if not depth_frame or not color_frame:
            return False, None, None
        return True, depth_frame, color_frame
    
    def get_depth_scale(self):
        """
        "scaling factor" refers to the relation between depth map units and meters; 
        it has nothing to do with the focal length of the camera.
        Depth maps are typically stored in 16-bit unsigned integers at millimeter scale, thus to obtain Z value in meters, the depth map pixels need to be divided by 1000.
        """
        return self.depth_scale

    def release(self):
        self.pipeline.stop()