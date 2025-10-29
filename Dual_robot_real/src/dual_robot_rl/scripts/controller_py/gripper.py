import rospy
from std_msgs.msg import Float64
from trajectory_msgs.msg import *
from control_msgs.msg import *
from move_demo.srv import GripperControl, GripperControlRequest  # Import the service
import actionlib
import copy


class Gripper(object):
    """ UR5 arm controller """

    def __init__(self, prefix):
        self._client = actionlib.SimpleActionClient(prefix+'gripper/follow_joint_trajectory', FollowJointTrajectoryAction)
        self._goal = FollowJointTrajectoryGoal()
        self.gripper_joint_position_publisher = rospy.Publisher(prefix+'gripper/command',
                                                    JointTrajectory,
                                                    queue_size=10)
        server_up = self._client.wait_for_server(timeout=rospy.Duration(10))
        if not server_up:
            rospy.logerr('Timed out waiting for Joint Trajectory'
                         ' Action Server to connect. Start the action server'
                         ' before running this node.')
            raise rospy.ROSException('JointTrajectoryController timed out: {0}'.format(prefix +'gripper/follow_joint_trajectory'))
        rospy.logdebug('Successfully connected to [%s]' % prefix +'gripper/follow_joint_trajectory')
        self._goal.trajectory.joint_names = ['gripper_finger1_joint']
        


    def add_point(self, positions, time, velocities=None, accelerations=None):
        """
        Adds a point to the trajectory. Each point must be specified by the goal position and 
        the goal time. The velocity and acceleration are optional.
        @type  positions: list
        @param positions: The goal position in the joint space
        @type  time: float
        @param time: The time B{from start} when the robot should arrive at the goal position.
        @type  velocities: list
        @param velocities: The velocity of arrival at the goal position. If not given zero 
        velocity is assumed.
        @type  accelerations: list
        @param accelerations: The acceleration of arrival at the goal position. If not given 
        zero acceleration is assumed.
        """
        point = JointTrajectoryPoint()
        point.positions = copy.deepcopy(positions)
        if type(velocities) == type(None):
            point.velocities = [0]
        else:
            point.velocities = copy.deepcopy(velocities)
        if type(accelerations) == type(None):
            point.accelerations = [0]
        else:
            point.accelerations = copy.deepcopy(accelerations)
        point.time_from_start = rospy.Duration(time)
        self._goal.trajectory.points.append(point)

    def clear_points(self):
        """
        Clear all points in the trajectory.
        """
        self._goal.trajectory.points = []

    def start(self, delay=0.1, wait=False):
        """
        Starts the trajectory. It sends the C{FollowJointTrajectoryGoal} to the action server.
        @type  delay: float
        @param delay: Delay (in seconds) before executing the trajectory
        """
        num_points = len(self._goal.trajectory.points)
        rospy.logdebug('Executing Joint Trajectory with {0} points'.format(num_points))
        self._goal.trajectory.header.stamp = rospy.Time.now() + rospy.Duration(delay)
        if wait:
            self._client.send_goal_and_wait(self._goal)
        else:
            self._client.send_goal(self._goal)

    def set_joint_positions(self,
                            position,
                            velocities=None,
                            accelerations=None,
                            wait=False,
                            t=5.0):
        self.add_point(positions=position,
                        time=t,
                        velocities=velocities,
                        accelerations=accelerations)
        self.start(delay=0, wait=wait)
        self.clear_points()
        return 'done'
    
    def _flexible_trajectory(self, position, time=5.0, vel=None):
        """ Publish point by point making it more flexible for real-time control """
        # Set up a trajectory message to publish.
        action_msg = JointTrajectory()
        action_msg.joint_names = ['gripper_finger1_joint']

        # Create a point to tell the robot to move to.
        target = JointTrajectoryPoint()
        target.positions = position

        # These times determine the speed at which the robot moves:
        if vel is not None:
            target.velocities = [vel]

        target.time_from_start = rospy.Duration(time)

        # Package the single point into a trajectory of points with length 1.
        action_msg.points = [target]

        self.gripper_joint_position_publisher.publish(action_msg)

if __name__ == "__main__":
    gripper = Gripper('left')
    gripper.gripper_joint_control(0)