import rospy
import rospkg
import roslaunch
import os

from std_srvs.srv import Trigger, TriggerResponse





class ROSLauncher(object):
    def __init__(self,
                 rospackage_name,
                 launch_file_name):
        self.launch = None
        self._rospackage_name = rospackage_name
        self._launch_file_name = launch_file_name

        self.rospack = rospkg.RosPack()

        # Check Package Exists
        pkg_path = self.rospack.get_path(rospackage_name)

        # If the package was found then we launch
        if pkg_path:
            rospy.loginfo("> Package found in workspace -->" + str(pkg_path))
            launch_dir = os.path.join(pkg_path, "launch")
            self.path_launch_file_name = os.path.join(launch_dir, launch_file_name)

            rospy.logwarn("path_launch_file_name==" +
                          str(self.path_launch_file_name))
            rospy.init_node('launch_manager_server')
 
            # Create the ROS service
            self.close_service = rospy.Service('/close_launch', Trigger, self.handle_close_request)
            self.start_service = rospy.Service('/start_launch', Trigger, self.handle_start_request)
            
        else:
            assert False, "No Package Path was found for ROS apckage ==>" + \
                str(rospackage_name)
    def handle_start_request(self,req):
        rospy.loginfo("Launch process is start.")
        self.start()
        return TriggerResponse(success=True, message="Launch process is running.")
 
 
    def handle_close_request(self, req):
        # Shutdown the launch process
        if self.launch is not None:
            self.launch.shutdown()
            rospy.loginfo("Launch process is shutdown.")
            return TriggerResponse(success=True, message="Launch process is shutdown.")
 
        return TriggerResponse(success=False, message="No launch process is running.")
    def start(self):
        self.uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
        roslaunch.configure_logging(self.uuid)
        self.launch = roslaunch.parent.ROSLaunchParent(
            self.uuid, [self.path_launch_file_name])
        self.launch.start()

        rospy.loginfo("> STARTED Roslaunch-->" +
                        str(self._launch_file_name))
            

if __name__ == '__main__':
    ros_launch = ROSLauncher('dual_gazebo','dual_robot_without_body.launch')
    ros_launch.start()
    rospy.spin()
    # rospy.sleep(10)
    # y = rospy.get_param_names()
    # for name in y:
    #     rospy.delete_param(name)
    # # y = rospy.get_published_topics()
    # print('finish')
    # rospy.signal_shutdown('nothing')




