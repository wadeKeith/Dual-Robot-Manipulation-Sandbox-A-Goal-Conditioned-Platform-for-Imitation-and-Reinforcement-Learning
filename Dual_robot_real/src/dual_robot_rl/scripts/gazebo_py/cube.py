from controller_py import transformations
from geometry_msgs.msg import (
    Pose,
    Point,
    Quaternion
)
from gazebo_msgs.srv import (
    SpawnModel,
    DeleteModel,
)
from gazebo_msgs.msg import ModelStates, ModelState
import rospkg
import rospy
from .basic_models import BOX, BOX_TARGET


class Cube(object):
    """ Model object """
    def __init__(self,model_type, pose, orientation):
        """
        Model representation for Gazebo spawner
        name string: name of the model as it is called in the sdf/urdf
        position array[3]: x, y, z position
        orienation array[4]: ax, ay, az, w
        file_type string: type of model sdf, urdf, or string
        string_model string: full xml representing a sdf model
        reference_frame string: frame of reference for the position/orientation of the model 
        """
        # self._pub_model_state = rospy.Publisher('/gazebo/set_model_state',ModelState, queue_size=10)
        if model_type =='cube':
            self.name = 'cube'
            self.posearr = pose
            self.set_pose(self.posearr, orientation)
            self.model_id = 'cube_tmp'
            self.reference_frame = 'desk_base_link'
            # get the file path for rospy_tutorials
            self.sdf_string = BOX
        elif model_type == 'cube_target':
            self.name = 'cube_target'
            self.posearr = pose
            self.set_pose(self.posearr, orientation)
            self.model_id = 'cube_target_tmp'
            self.reference_frame = 'desk_base_link'
            # get the file path for rospy_tutorials
            self.sdf_string = BOX_TARGET

    def set_pose(self, position, orientation=[0,0,0,1]):
        self.posearr = position
        if len(orientation) == 3:
            self.orietarr = transformations.quaternion_from_euler(orientation[0], orientation[1], orientation[2])
        else:
            self.orietarr = orientation
        self.orientation = Quaternion(x=self.orietarr[0], y=self.orietarr[1], z=self.orietarr[2], w=self.orietarr[3]) if isinstance(orientation, list) else Quaternion()
        self.pose = Pose(position=Point(x=position[0], y=position[1], z=position[2]), orientation=self.orientation)

    def get_rotation(self):
        return self.orietarr

    def get_pose(self):
        return self.posearr
    
    def delete_model(self):
        # rospy.wait_for_service('/gazebo/spawn_sdf_model')
        delete_sdf = rospy.ServiceProxy('/gazebo/delete_model', DeleteModel,persistent=True)
        # rospy.sleep(0.5)
        delete_sdf.call(self.model_id)
        delete_sdf.close()
        del delete_sdf
        # delete_gazebo_models([self.model_id])
    
    def spawn_model(self):
        rospy.wait_for_service('/gazebo/spawn_sdf_model')
        try:
            spawn_sdf = rospy.ServiceProxy('/gazebo/spawn_sdf_model', SpawnModel)
            spawn_sdf.call(self.model_id, self.sdf_string, "/",
                        self.pose, self.reference_frame)
            spawn_sdf.close()
            del spawn_sdf
        except rospy.ServiceException as e:
            rospy.logerr("Spawn SDF service call failed: {0}".format(e))
