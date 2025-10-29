import rospy
import rospkg
import roslaunch
import os




class ROSLauncher(object):
    def __init__(self,
                 rospackage_name,
                 launch_file_name,
                 ros_ws_abspath="/home/user/simulation_ws"):

        self._rospackage_name = rospackage_name
        self._launch_file_name = launch_file_name

        self.rospack = rospkg.RosPack()

        # Check Package Exists
        pkg_path = self.rospack.get_path(rospackage_name)

        # If the package was found then we launch
        if pkg_path:
            rospy.loginfo("> Package found in workspace -->" + str(pkg_path))
            launch_dir = os.path.join(pkg_path, "launch")
            path_launch_file_name = os.path.join(launch_dir, launch_file_name)

            rospy.logwarn("path_launch_file_name==" +
                          str(path_launch_file_name))

            self.uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
            roslaunch.configure_logging(self.uuid)
            self.launch = roslaunch.parent.ROSLaunchParent(
                self.uuid, [path_launch_file_name])
            self.launch.start()

            rospy.loginfo("> STARTED Roslaunch-->" +
                          str(self._launch_file_name))
        else:
            assert False, "No Package Path was found for ROS apckage ==>" + \
                str(rospackage_name)
            

if __name__ == '__main__':
    # ros_launch = ROSLauncher('dual_gazebo','view_dual_ur5e_model.launch')
    # rospy.sleep(10)
    y = rospy.get_param_names()
    for name in y:
        rospy.delete_param(name)
    # y = rospy.get_published_topics()
    print('finish')
    rospy.signal_shutdown('nothing')




