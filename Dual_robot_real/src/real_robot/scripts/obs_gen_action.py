import numpy as np
import rospy
from real_robot_right import Robot_right,goal_distance


init_point = [0, -1.396, 1.396, -1.57, 2.355, 0]
mid_point = np.deg2rad([-65.55, -28.3, 94.19, -73.2, 27.33, 49.88])
click_point = np.deg2rad([-43.27, -3.91, 33.1, -33.41, 50.57, 46.32])
catch_point = np.deg2rad([-43.71, -6.45, 44.64, -42.6, 47.88, 45.93])
final_point = np.deg2rad([-33.63, -58.5, 104.6, -8.13, 101.54, 36.43])

obs = np.load('obs_dict.npy', allow_pickle=True).item()

pos_ls = obs['pos_ls']
vel_ls = obs['vel_ls']
eff_ls = obs['eff_ls']

start_idx = 6
end_idx = np.linalg.norm(pos_ls-final_point,axis=1).argmin()

pos_ls = pos_ls[start_idx:end_idx]
vel_ls = vel_ls[start_idx:end_idx]
eff_ls = eff_ls[start_idx:end_idx]


action = np.concatenate((pos_ls, vel_ls, eff_ls), axis=1)[1:]

gripper_action = np.zeros([action.shape[0],1])

mid_point_idxs = np.linalg.norm(action[:,:6]-mid_point,axis=1).argpartition(2)[:2]
# mid_point_idxs = np.array([27,49])
mid_point_idx_first = min(mid_point_idxs)
mid_point_idx_second = max(mid_point_idxs)

click_point_idx = np.linalg.norm(action[:,:6]-click_point,axis=1).argmin()
catch_point_idx = np.linalg.norm(action[:,:6]-catch_point,axis=1).argpartition(8)[:8]

catch_point_idx[2] = 63
catch_point_idx[7] = 64
catch_point_idx[5] = 65
gripper_action[:mid_point_idx_first+1] = -1
gripper_action[click_point_idx:mid_point_idx_second+1] = 1
gripper_action[catch_point_idx] = -1

action_list = np.concatenate((action,gripper_action),axis=1)

final_action = action_list[-1,:]
final_action[-1] = 1
final_action = np.repeat([final_action],10,axis=0)
action_list = np.concatenate((action_list, final_action),axis=0)

rospy.init_node('right_robot',
                anonymous=True)
robot = Robot_right()





# gripper_control = np.zeros([action_list.shape[0],1])

# action_list = np.concatenate([action_list, gripper_control],axis=1)

obs = robot.reset()

for i in range(len(action_list)):
    obs = robot.step(action_list[i])
    # print('a')

robot.right_arm.stop_service_client()
rospy.sleep(0.5)


print('a')













