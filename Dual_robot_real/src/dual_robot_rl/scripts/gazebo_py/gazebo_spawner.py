import copy
import rospy
import rospkg

from gazebo_msgs.msg import ModelStates, ModelState
from gazebo_msgs.srv import (
    SpawnModel,
    DeleteModel,
)


def delete_gazebo_models(models):
    # This will be called on ROS Exit, deleting Gazebo models
    # Do not wait for the Gazebo Delete Model service, since
    # Gazebo should already be running. If the service is not
    # available since Gazebo has been killed, it is fine to error out  
    for model in models:
        rospy.ServiceProxy('/gazebo/delete_model', DeleteModel).call(model)

class GazeboModels:
    """ Class to handle ROS-Gazebo model respawn """

    def __init__(self, model_pkg):
        # self._pub_model_state = rospy.Publisher('/gazebo/set_model_state',
        #                                         ModelState, queue_size=10)
        # Get Models' Path
        # get an instance of RosPack with the default search paths
        rospack = rospkg.RosPack()
        # get the file path for rospy_tutorials
        packpath = rospack.get_path(model_pkg)
        self.model_path = packpath + '/models/'

    def load_models(self, models):
        for m in models:
            if m.file_type == 'urdf':
                self.load_urdf_model(m)
            elif m.file_type == 'sdf' or m.file_type == 'string':
                self.load_sdf_model(m)


    def load_urdf_model(self, model): #need to modify, dont use it
        # Spawn Block URDF
        rospy.wait_for_service('/gazebo/spawn_urdf_model')
        try:
            m_id = model.model_id
            spawn_urdf = rospy.ServiceProxy('/gazebo/spawn_urdf_model', SpawnModel)
            spawn_urdf(m_id, self.load_xml(model.name, filetype="urdf"), "/",
                       model.pose, model.reference_frame)
        except IOError:
            self.load_sdf_model(model)
        except rospy.ServiceException as e:
            rospy.logerr("Spawn URDF service call failed: {0}".format(e))

    def load_sdf_model(self, model): #这里可能会有点慢，如果有点慢可以把sdf文件直接读出来后保存在self里面，不用每次都读一次
        # Spawn model SDF
        rospy.wait_for_service('/gazebo/spawn_sdf_model')
        try:
            spawn_sdf = rospy.ServiceProxy('/gazebo/spawn_sdf_model', SpawnModel)
            m_id = model.model_id
            if model.string_model is None:
                spawn_sdf(m_id, self.load_xml(model.name), "/",
                          model.pose, model.reference_frame)
            else:
                spawn_sdf(m_id, model.string_model, "/",
                          model.pose, model.reference_frame)
        except rospy.ServiceException as e:
            rospy.logerr("Spawn SDF service call failed: {0}".format(e))

    def load_xml(self, model_name, filetype="sdf"):
        # Load File
        with open(self.model_path + model_name + "/model.%s" % filetype, "r") as table_file:
            return table_file.read().replace('\n', '')
