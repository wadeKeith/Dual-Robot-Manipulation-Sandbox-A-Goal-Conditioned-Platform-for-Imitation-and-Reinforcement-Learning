import rospy
from sensor_msgs.msg import JointState
import numpy as np

valid_joint_names = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint', 'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']
obs_ls = []
pos_ls = []
vel_ls = []
eff_ls = []


def demo_cb(msg):
    position = []
    velocity = []
    effort = []
    name = []
    for joint_name in valid_joint_names:
        if joint_name in msg.name:
            idx = msg.name.index(joint_name)
            name.append(msg.name[idx])
            if msg.effort:
                effort.append(msg.effort[idx])
            if msg.velocity:
                velocity.append(msg.velocity[idx])
            position.append(msg.position[idx])
    if set(name) == set(valid_joint_names):
        current_jnt_positions = np.array(position)
        pos_ls.append(current_jnt_positions)
        current_jnt_velocities = np.array(velocity)
        vel_ls.append(current_jnt_velocities)
        current_jnt_efforts = np.array(effort)
        eff_ls.append(current_jnt_efforts)
        joint_names = list(name)
    rospy.sleep(0.22)

rospy.init_node('gen_demo', anonymous=True)

demo_sub = rospy.Subscriber('/right/joint_states', JointState, demo_cb, queue_size=1)
while not rospy.is_shutdown():
    rospy.sleep(0)
obs_dict = {'pos_ls': np.array(pos_ls),
            'vel_ls': np.array(vel_ls),
            'eff_ls': np.array(eff_ls)}
np.save('obs_dict',obs_dict)
rospy.loginfo('save!')









