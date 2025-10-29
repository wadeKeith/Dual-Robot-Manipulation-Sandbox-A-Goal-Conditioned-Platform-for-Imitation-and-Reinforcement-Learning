import rospy
from std_msgs.msg import Float64
from trajectory_msgs.msg import *
from control_msgs.msg import *
from gripper_msg.Robotiq2FGripper_robot_output import Robotiq2FGripper_robot_output
from gripper_msg.Robotiq2FGripper_robot_input import Robotiq2FGripper_robot_input
import actionlib
import numpy as np

def gripper_pos2width(pos):
    return 1-int(pos)/255

class Gripper2f(object):
    """ UR5 arm controller """

    def __init__(self):
        self.joint_position_publisher = rospy.Publisher("Robotiq2FGripperRobotOutput", Robotiq2FGripper_robot_output, queue_size=10)
        self._js_sub = rospy.Subscriber("Robotiq2FGripperRobotInput", Robotiq2FGripper_robot_input, self.joint_states_cb)
        self.command = Robotiq2FGripper_robot_output()
        
    def reset(self):
        # reset
        self.command.rACT = 0
        self.joint_position_publisher.publish(self.command)
        rospy.sleep(0.1)
        # activate gripper
        self.command.rACT = 1
        self.command.rGTO = 1
        self.command.rSP = 255
        self.command.rFR = 150
        self.joint_position_publisher.publish(self.command)
        rospy.sleep(2)
    def joint_states_cb(self,status):
        self._gACT = status.gACT
        self._gGTO = status.gGTO
        self._gSTA = status.gSTA
        self._gOBJ = status.gOBJ
        self._gFLT = status.gFLT
        self._gPR = status.gPR
        self._gPO = status.gPO
        self._gCU = status.gCU
    def get_gripper_pos(self):
        return np.array(gripper_pos2width(self._gPR))
    
    def set_joint_positions_flex(self,width,during_time):
        width = np.clip(width,0,1)
        assert width<=1 and width>=0
        pos = int((1-width)*255)
        self.command.rPR = pos
        self.joint_position_publisher.publish(self.command)
        # rospy.sleep(during_time)


if __name__ == "__main__":
    rospy.init_node('gripper_test',
                anonymous=True)
    gripper = Gripper2f()
    gripper.reset()

    
    gripper_pos = gripper.get_gripper_pos()
    print(gripper_pos)
    gripper.set_joint_positions_flex(0.5,during_time = 0)
    rospy.sleep(0.5)
    gripper_pos = gripper.get_gripper_pos()
    print(gripper_pos)
