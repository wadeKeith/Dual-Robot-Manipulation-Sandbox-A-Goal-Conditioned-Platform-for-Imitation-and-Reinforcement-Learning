IKFAST = 'ikfast'
TRAC_IK = 'trac_ik'

JOINT_PUBLISHER_ROBOT = 'arm_controller'
JOINT_SUBSCRIBER = '/arm_controller/state'
JOINT_STATE_SUBSCRIBER = '/joint_states'
FT_SUBSCRIBER = 'wrench'

GENERIC_GRIPPER ='simple'
ROBOTIQ_GRIPPER ='85'

# Set constants for joints


BASE_LINK = 'world'
EE_LINK = 'gripper_tcp'
FT_LINK = 'gripper_tcp'



JOINT_ORDER = [
    'shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint', 'wrist_1_joint',
    'wrist_2_joint', 'wrist_3_joint'
]
def get_arm_joint_names(prefix):
    return [prefix + joint for joint in JOINT_ORDER]

# RESULT_CODE
DONE = 'done'
FORCE_TORQUE_EXCEEDED = 'force_exceeded'
STOP_ON_TARGET_FORCE = 'stop_on_target_force'
IK_NOT_FOUND = 'ik_not_found'
SPEED_LIMIT_EXCEEDED = 'speed_limit_exceeded'
TERMINATION_CRITERIA = 'termination_criteria_achieved'
