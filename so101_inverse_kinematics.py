import numpy as np
from so101_forward_kinematics import *

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
    y_des, x_des, _ = target_position
    offset = 0.0388353
    theta_1 = np.arctan2(x_des, y_des - offset)
    
    joint_config['shoulder_pan'] = np.rad2deg(-theta_1)

    # solve for theta_3
    a_1 = 0.0542 + 0.0624
    a_2 = 0.11257
    a_3 = 0.1349
    offset = 0.0388353 + 0.0303992 + 0.028
    wrist_flex_position, _ = get_wrist_flex_position(target_position)
    x, y, z_3 = wrist_flex_position
    x_3 = np.sqrt(x**2 + y**2) - offset

    r = np.sqrt(x_3**2 + (z_3 - a_1)**2)
    phi_2 = np.arccos((a_2**2 + a_3**2 - r**2) / (2*a_2*a_3) )
    theta_3 = np.pi/2 - phi_2

    joint_config['elbow_flex'] = np.rad2deg(theta_3)

    # solve for theta_2
    phi_3 = np.arctan(x_3 / (z_3 - a_1))
    phi_1 = np.arccos((a_2**2 + r**2 - a_3**2) / (2*a_2*r))
    theta_2 = phi_3 - phi_1

    joint_config['shoulder_lift'] = np.rad2deg(theta_2)

    # solve for theta_4
    gw1 = get_gw1(joint_config['shoulder_pan'])
    g12 = get_g12(joint_config['shoulder_lift'])
    g23 = get_g23(joint_config['elbow_flex'])
    g34 = get_g34(joint_config['wrist_flex'])
    gw4 = gw1 @ g12 @ g23 @ g34
    current_wrist_orientation = gw4[0:3, 0:3]

    _, desired_wrist_orientation = get_wrist_flex_position(target_position)
    
    z_x_current, z_y_current, _ = current_wrist_orientation[2]
    z_x_desired, z_y_desired, _ = desired_wrist_orientation[2]

    theta_4 = np.arctan2(z_y_current, z_x_current) - np.arctan2(z_y_desired, z_x_desired) #! why??

    joint_config['wrist_flex'] = np.rad2deg(theta_4)

    # solve for theta_5
    theta_5 = joint_config['shoulder_pan']
    joint_config['wrist_roll'] = theta_5

    return joint_config

def get_wrist_flex_position(target_position):
    gwt = np.block([[np.identity(3), np.array(target_position).reshape(3,1)], [0, 0, 0, 1]])
    g4t = get_g45(0) @ get_g5t()
    gw4 = gwt @ np.linalg.inv(g4t)
    wrist_flex_position = gw4[0:3, 3]
    wrist_flex_orientation = gw4[0:3, 0:3]
    return wrist_flex_position, wrist_flex_orientation
