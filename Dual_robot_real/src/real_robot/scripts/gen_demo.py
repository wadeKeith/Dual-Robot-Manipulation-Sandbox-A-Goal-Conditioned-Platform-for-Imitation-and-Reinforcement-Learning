from real_robot_right import Robot_right,goal_distance
import rospy
import numpy as np
from utils.utils import PID
import cv2

class GenDemo(object):

    def __init__(self):
        self.robot = Robot_right()
        K_p = 1*np.ones(7)
        K_i = 0.001*np.ones(7)
        K_d = 0.06*np.ones(7)
        self.pid_controller = PID(Kp = K_p, Ki=K_i, Kd=K_d, dynamic_pid=False, max_gain_multiplier=1)
        self.obs_ls = []
        self.action_list = []
        self.run_step = 0
    
    def reset(self,waypoint):
        self.waypoint_click_button_coarse = waypoint['waypoint_click_button_coarse']
        self.waypoint_click_button_grind = waypoint['waypoint_click_button_grind']
        self.waypoint_grasp_handle_coarse = waypoint['waypoint_grasp_handle_coarse']
        self.waypoint_grasp_handle_grind = waypoint['waypoint_grasp_handle_grind']
        self.waypoint_open_door = waypoint['waypoint_open_door']
        obs = self.robot.reset()
        self.obs_ls.append(obs)
        

    def gen(self):
        self.follow_traj(self.waypoint_click_button_coarse,self.obs_ls[-1], 0.5)
        self.follow_traj(self.waypoint_click_button_grind,self.obs_ls[-1], 0.04)
        self.follow_traj(self.waypoint_grasp_handle_coarse,self.obs_ls[-1], 0.5)
        self.follow_traj(self.waypoint_grasp_handle_grind,self.obs_ls[-1], 0.04)
        self.gripper_move(-1,5)
        self.follow_traj(self.waypoint_open_door,self.obs_ls[-1], 0.04,False)
        self.gripper_move(1,10)
        

    def follow_traj(self,traj,init_obs,acc_factor,set_gripper=True):
        obs = init_obs
        for i in range(len(traj)):
            if set_gripper:
                while goal_distance(np.concatenate((obs['agent_pos'][:6],obs['agent_pos'][-1:])),traj[i])>acc_factor:
                    
                    action = np.clip(self.pid_controller.update(traj[i]- np.concatenate((obs['agent_pos'][:6],obs['agent_pos'][-1:]))),-1,1)
                    self.action_list.append(action.copy())
                    obs = self.robot.step(action)
                    self.obs_ls.append(obs)
                    self.run_step += 1
            else:
                while goal_distance(obs['agent_pos'][:6],traj[i])>acc_factor:
        
                    action = np.clip(self.pid_controller.update(np.concatenate([traj[i]- obs['agent_pos'][:6],np.array(0).reshape(-1,)])),-1,1)
                    self.action_list.append(action.copy())
                    obs = self.robot.step(action)
                    self.obs_ls.append(obs)
                    self.run_step += 1

    def gripper_move(self,cmd,time):
        for i in range(time):
            action = np.array([0,0,0,0,0,0,cmd])
            self.action_list.append(action.copy())
            obs = self.robot.step(action)
            self.obs_ls.append(obs)
            self.run_step += 1


if __name__ == "__main__":
    rospy.init_node('right_robot',
                anonymous=True)
    gen_demo = GenDemo()


    waypoint_click_button = np.array([[-0.19722221, -1.36414934,  1.71618225, -0.54209927,  2.12179677, 0.44820055,0.5],
                        [-0.74490652, -0.86376345,  1.80676484, -0.64175757,  1.62385434,0.7267551, 0],
                        [-1.16134208, -0.49497538,  1.67411982, -1.31754905,  0.5126032 ,0.88592913,0], # terminal state
                        [-0.95225164, -0.38362337,  1.27426489, -0.98122411,  0.66113072,0.82868233, 0],
                        [-0.77161006, -0.05393067,  0.52063172, -0.5412266 ,  0.826937  , 0.80686571,0]]) 

    waypoint_grasp_handle = np.array([[-1.16134208, -0.49497538,  1.67411982, -1.31754905,  0.5126032 ,0.88592913,1],
                                [-0.92572264, -0.33876841,  1.36380028, -1.11666166,  0.67422069,0.82013022, 1],
                                [-0.77038833, -0.14381513,  0.84543749, -0.77911498,  0.82833326,0.80232786, 0.5]])
    waypoint_open_door = np.array([[-0.81402156, -0.41032691,  1.3252285 , -0.77213366,  0.95801123,0.6695083],
                                [-0.81437063, -0.61191244,  1.57393792, -0.62779493,  1.10968034,0.6000442],
                                [-0.77091193, -0.83322018,  1.85441233, -0.62098815,  1.34303086,0.53372169],
                                [-0.6991789 , -1.04073983,  1.93749   , -0.33283429,  1.53466801,0.54908058],
                                #    [-0.33108896, -1.07774081,  1.63519898,  0.46896997,  2.250253  , 0.98209677]
                                ])

    waypoint_click_button_coarse = waypoint_click_button[:3]
    waypoint_click_button_grind = waypoint_click_button[3:]
    waypoint_grasp_handle_coarse = waypoint_grasp_handle[:1]
    waypoint_grasp_handle_grind = waypoint_grasp_handle[1:]

    waypoint = {'waypoint_click_button_coarse':waypoint_click_button_coarse,
                'waypoint_click_button_grind':waypoint_click_button_grind,
                'waypoint_grasp_handle_coarse':waypoint_grasp_handle_coarse,
                'waypoint_grasp_handle_grind':waypoint_grasp_handle_grind,
                'waypoint_open_door':waypoint_open_door}
    
    gen_demo.reset(waypoint)
    gen_demo.gen()
    print(gen_demo.run_step)
    np.save('action_list',gen_demo.action_list)
    np.save('obs', gen_demo.obs_ls)
    gen_demo.robot.right_arm.stop_service_client()
    rospy.sleep(0.5)

    top_camera = cv2.VideoWriter('top_camera.mp4', cv2.VideoWriter_fourcc(*'mp4v'), 10, (640,480))
    right_camera = cv2.VideoWriter('right_camera.mp4', cv2.VideoWriter_fourcc(*'mp4v'), 10, (640,480))


    for i in range(len(gen_demo.obs_ls)):
        top_camera.write(gen_demo.obs_ls[i]['top_img'])
        right_camera.write(gen_demo.obs_ls[i]['right_img'])

    top_camera.release()
    right_camera.release()




























