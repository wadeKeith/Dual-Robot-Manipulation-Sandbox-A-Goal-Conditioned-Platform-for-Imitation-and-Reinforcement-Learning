import rospy
from std_srvs.srv import Trigger
from trajectory_msgs.msg import *
from control_msgs.msg import *
import actionlib
import numpy as np
from .real_controllers import JointTrajectoryController

from .ur_pykdl.ur_pykdl import ur_kinematics
# try:
#     from ur_ikfast import ur_kinematics as ur_ikfast
# except ImportError:
#     print("Import ur_ikfast not available, IKFAST would not be supported without it")
from trac_ik_python.trac_ik import IK
import utils
from . import transformations
IKFAST = 'ikfast'
TRAC_IK = 'trac_ik'
IK_NOT_FOUND = 'ik_not_found'


class Real_arm(object):

    def __init__(self,prefix):
        self.prefix = prefix
        self.base_link = 'world'
        self.ee_link = 'gripper_tcp'
        self.joint_names = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint', 'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']
        self.max_joint_speed = np.deg2rad([180, 180, 180, 180, 180, 180])
        
        self.power_on_service_client = rospy.ServiceProxy(prefix+'ur_hardware_interface/dashboard/power_on', Trigger)
        self.brake_release_service_client = rospy.ServiceProxy(prefix+'ur_hardware_interface/dashboard/brake_release', Trigger)
        self.play_service_client = rospy.ServiceProxy(prefix+'ur_hardware_interface/dashboard/play', Trigger)
        self.stop_service_client = rospy.ServiceProxy(prefix+'ur_hardware_interface/dashboard/stop', Trigger)

        self._init_controllers()
        self.ik_solver = 'trac_ik' # 'ikfast'
        self._init_ik_solver(self.base_link, self.ee_link, self.prefix)
    
    def _init_controllers(self):

        # Flexible trajectory (point by point)
        self.joint_position_publisher = rospy.Publisher(self.prefix+'arm_controller/command',
                                                    JointTrajectory,
                                                    queue_size=10)

        self.joint_traj_controller = JointTrajectoryController(
            publisher_name='arm_controller', namespace=self.prefix, joint_names=self.joint_names, timeout=10.0)


    def _init_ik_solver(self, base_link, ee_link, namespace):
        assert rospy.has_param(namespace+"robot_description") == True
        self.kdl = ur_kinematics(base_link=base_link, ee_link=ee_link,namespace=namespace)
        
        # if self.ik_solver == IKFAST:
        #     # IKfast libraries
        #     try:
        #         self.arm_ikfast = ur_ikfast.URKinematics(self._robot_urdf)
        #     except Exception:
        #         rospy.logerr("IK solver set to IKFAST but no ikfast found for: %s. Switching to TRAC_IK" % self._robot_urdf)
        #         self.ik_solver == TRAC_IK
        #         return self._init_ik_solver(base_link, ee_link)
        assert self.ik_solver == TRAC_IK
        try:
            # if not rospy.has_param(namespace+"robot_description"):
            #     self.trac_ik = IK(base_link=base_link, tip_link=ee_link, solve_type="Distance", timeout=0.002, epsilon=1e-5,
            #                       urdf_string=utils.load_urdf_string(self._robot_urdf_package, self._robot_urdf))
            assert rospy.has_param(namespace+"robot_description") == True
            urdf_string = rospy.get_param(namespace+"robot_description")
            self.trac_ik = IK(base_link=base_link, tip_link=ee_link, solve_type="Distance",urdf_string=urdf_string)
        except Exception as e:
            rospy.logerr("Could not instantiate TRAC_IK" + str(e))
        # else:
        #     raise Exception("unsupported ik_solver", self.ik_solver)

    def arm_init(self):
        self.power_on_service_client()
        rospy.sleep(20)
        self.brake_release_service_client()
        rospy.sleep(20)
        # self.play_service_client()
        # rospy.sleep(1)
        # self.stop_service_client()

    def _solve_ik(self, pose, q_guess=None, attempts=5, verbose=True):
        q_guess_ = q_guess if q_guess is not None else self.joint_angles()
        # TODO(cambel): weird it shouldn't happen but...
        if isinstance(q_guess, np.float64):
            q_guess_ = None

        if self.ik_solver == IKFAST:
            ik = self.arm_ikfast.inverse(pose, q_guess=q_guess_)

        elif self.ik_solver == TRAC_IK:
            ik = self.trac_ik.get_ik(q_guess_, *pose)
            if ik is None:
                if attempts > 0:
                    return self._solve_ik(pose, q_guess, attempts-1)
                if verbose:
                    rospy.logwarn("TRACK-IK: solution not found!")

        return ik
    
    def end_effector(self,
                     joint_angles=None,
                     rot_type='quaternion',
                     tip_link=None):
        """ Return End Effector Pose """

        joint_angles = self.joint_angles() if joint_angles is None else joint_angles

        if rot_type == 'quaternion':
            # forward kinematics
            return self.kdl.forward(joint_angles, tip_link)

        elif rot_type == 'euler':
            x = self.end_effector(joint_angles)
            euler = np.array(transformations.euler_from_quaternion(x[3:], axes='rxyz'))
            return np.concatenate((x[:3], euler))

        else:
            raise Exception("Rotation Type not supported", rot_type)
    
    def set_joint_positions(self,
                            position,
                            velocities=None,
                            accelerations=None,
                            wait=False,
                            t=5.0):
        self.joint_traj_controller.add_point(positions=position,
                                             time=t,
                                             velocities=velocities,
                                             accelerations=accelerations)
        self.joint_traj_controller.start(delay=0, wait=wait)
        self.joint_traj_controller.clear_points()
        return 'done'

    def set_target_pose_flex(self, pose, t=5.0):
        """ Supported pose is only x y z aw ax ay az """
        q = self._solve_ik(pose)
        if q is None:
            # IK not found
            return IK_NOT_FOUND
        else:
            return self.set_joint_positions_flex(q, t=t)
    
    def set_joint_positions_flex(self, position, t=5.0, v=None, eff=None):
        qc = self.joint_angles()
        deltaq = (qc - position)
        speed = deltaq / t
        cmd = position
        if np.any(np.abs(speed) > self.max_joint_speed):
            # speed_clip = np.clip(speed,-self.max_joint_speed,self.max_joint_speed)
            # cmd = qc-t*speed_clip
            rospy.logwarn("Exceeded max speed %s deg/s, ignoring command" % np.round(np.rad2deg(speed), 0))
            return 'speed_limit_exceeded'
        self._flexible_trajectory(cmd, t, v, eff)
        return 'done'
    
    def _flexible_trajectory(self, position, time=5.0, vel=None, eff= None):
        """ Publish point by point making it more flexible for real-time control """
        # Set up a trajectory message to publish.
        action_msg = JointTrajectory()
        action_msg.joint_names = self.joint_names

        # Create a point to tell the robot to move to.
        target = JointTrajectoryPoint()
        target.positions = position

        # These times determine the speed at which the robot moves:
        if vel is not None:
            target.velocities = vel

        if eff is not None:
            target.effort = eff
        target.time_from_start = rospy.Duration(time)

        # Package the single point into a trajectory of points with length 1.
        action_msg.points = [target]

        self.joint_position_publisher.publish(action_msg)
        # print(target)

    def joint_angle(self, joint):
        """
        Return the requested joint angle.

        @type joint: str
        @param joint: name of a joint
        @rtype: float
        @return: angle in radians of individual joint
        """
        return self.joint_traj_controller.get_joint_positions()[joint]

    def joint_angles(self):
        """
        Return all joint angles.

        @rtype: dict({str:float})
        @return: unordered dict of joint name Keys to angle (rad) Values
        """
        return self.joint_traj_controller.get_joint_positions()

    def joint_velocity(self, joint):
        """
        Return the requested joint velocity.

        @type joint: str
        @param joint: name of a joint
        @rtype: float
        @return: velocity in radians/s of individual joint
        """
        return self.joint_traj_controller.get_joint_velocities()[joint]

    def joint_velocities(self):
        """
        Return all joint velocities.

        @rtype: dict({str:float})
        @return: unordered dict of joint name Keys to velocity (rad/s) Values
        """
        return self.joint_traj_controller.get_joint_velocities()
    
    def joint_efforts(self):
        return self.joint_traj_controller.get_joint_efforts()
    

if __name__ == "__main__":
    rospy.init_node('dual_robot',
                anonymous=True)
    action_time = 10
    robot = Real_arm(prefix='/right/')
    # robot.arm_init()
    robot.stop_service_client()
    rospy.sleep(1)
    robot.play_service_client()
    rospy.sleep(1)
    positions = [0, -0.785, 0.785, -1.57, 2.355, 0]
    robot._flexible_trajectory(positions,time=action_time)
    rate = rospy.Rate(1/action_time)
    rate.sleep()
    # rospy.spin()





# ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint', 'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']
# [0.17936818301677704, -2.6100555859007777, -1.8553049564361572, 4.575779123897217, 3.0, 0.002471494721248746]

# ['elbow_joint', 'shoulder_lift_joint', 'shoulder_pan_joint', 'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']
# [-1.8553049564361572, -2.6100555859007777, 0.17936818301677704, 4.575779123897217, 3.526235342025757, 0.002471494721248746]