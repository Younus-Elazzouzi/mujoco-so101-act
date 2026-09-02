import numpy as np
from so101_forward_kinematics import get_g45, get_g5t

def get_inverse_kinematics(target_position, target_orientation):
    "Geometric appraoch specific to the so-101 arms"
    
    # Initialize the joint configuration dictionary
    joint_config = {
        'shoulder_pan': 0.0,
        'shoulder_lift': 0.0,
        'elbow_flex': 0.0,
        'wrist_flex': 0.0,
        'wrist_roll': 0.0,
        'gripper': 0.0
    }

    # solve for theta_1
    x_des , y_des ,_ = target_position
    offset = 0.0388353
    a = np.atan2(y_des - offset, x_des)
    theta_1 = np.rad2deg(np.pi/2 - a)
    
    joint_config['shoulder_pan'] = -theta_1

    return joint_config

def get_wrist_flex_position(target_position):
    gwt = np.block([[np.identity(3), np.array(target_position).reshape(3,1)], [0, 0, 0, 1]])
    g4t = get_g45(0) @ get_g5t()
    gw4 = gwt @ np.linalg.inv(g4t)
    wrist_flex_position = gw4[0:3, 3]
    wrist_flex_orientation = gw4[0:3, 0:3]
    return wrist_flex_position, wrist_flex_orientation
