import rospy
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
import actionlib
from trajectory_msgs.msg import JointTrajectoryPoint, JointTrajectory
import copy


def reset(prefix,positions):
    topic ='/' + prefix + 'pos_joint_traj_controller/command'
    client = rospy.Publisher(topic, JointTrajectory, queue_size=1)
    cmd = JointTrajectory()

    point = JointTrajectoryPoint()
    point.positions = copy.deepcopy(positions)
    point.velocities = [0]*6
    point.time_from_start = rospy.Duration(0.05)
    cmd.points.append(point)
    cmd.header.stamp = rospy.Time.now()
    client.publish(cmd)
    rospy.logwarn('Success to send the cmd')


# left_qc = [-0.1568, 3.5337, -0.4198, -1.467, -2.2789, 0.00]
# right_qc = [0.1568, -0.5444, 0.6135, -1.7622, 2.3343, 0.00]
left_qc = [0]*6
right_qc = [0]*6
rospy.init_node('init_all',
                anonymous=True, log_level=rospy.WARN)

reset('left_', left_qc)
reset('right_', right_qc)
rospy.logwarn('finish!!!')











