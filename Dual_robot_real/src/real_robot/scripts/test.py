import rospy
from real_robot_right import Robot_right,goal_distance
import rospy
import numpy as np


rospy.init_node('right_robot',
                anonymous=True)
robot = Robot_right()



action_list = np.load('obs.npy',allow_pickle=True)

# gripper_control = np.zeros([action_list.shape[0],1])

# action_list = np.concatenate([action_list, gripper_control],axis=1)

obs = robot.reset()

for i in range(1,len(action_list)):
    obs = robot.step(action_list[i]['agent_pos'])
    # print('a')

robot.right_arm.stop_service_client()
rospy.sleep(0.5)

