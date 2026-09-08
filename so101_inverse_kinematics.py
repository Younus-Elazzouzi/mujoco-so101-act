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

    # solve for theta_3
    a_1 = 0.0542 + 0.0624
    a_2 = 0.11257
    a_3 = 0.1349
    wrist_flex_position, _ = get_wrist_flex_position(target_position)
    x_3, _, z_3 = wrist_flex_position

    r = np.sqrt(x_3**2 + (z_3 - a_1)**2)
    phi_2 = np.arccos((a_2**2 + a_3**2 - r**2) / (2*a_2*a_3) )
    theta_3 = np.pi/2 - phi_2

    joint_config['elbow_flex'] = np.rad2deg(theta_3)

    # solve for theta_2
    phi_3 = np.arctan(x_3 / (z_3 - a_1))
    phi_1 = np.arccos((a_2**2 + r**2 - a_3**2) / (2*a_2*r))
    theta_2 = phi_3 - phi_1

    joint_config['shoulder_lift'] = np.rad2deg(theta_2)



    return joint_config

def get_wrist_flex_position(target_position):
    gwt = np.block([[np.identity(3), np.array(target_position).reshape(3,1)], [0, 0, 0, 1]])
    g4t = get_g45(0) @ get_g5t()
    gw4 = gwt @ np.linalg.inv(g4t)
    wrist_flex_position = gw4[0:3, 3]
    wrist_flex_orientation = gw4[0:3, 0:3]
    return wrist_flex_position, wrist_flex_orientation
