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

    # solve for theta_2 and theta_3
    wrist_flex_position, _ = get_wrist_flex_position(target_position)
    wrist_flex_position = np.asarray(wrist_flex_position, dtype=float).reshape(3)

    gw2_zero = get_gw1(joint_config["shoulder_pan"]) @ get_g12(0.0)
    p2 = gw2_zero[:3, :3].T @ (wrist_flex_position - gw2_zero[:3, 3])
    x, y = p2[:2]

    l1 = np.hypot(0.11257, 0.028)
    l2 = np.hypot(0.1349, 0.0052)

    r = np.hypot(x, y)

    if r < abs(l1 - l2) - 1e-9 or r > l1 + l2 + 1e-9:
        raise ValueError("Target position is out of reach for the arm.")

    cos_beta = (l1**2 + l2**2 - r**2) / (2 * l1 * l2)
    cos_alpha = (r**2 + l1**2 - l2**2) / (2 * l1 * r)

    beta = np.arccos(np.clip(cos_beta, -1.0, 1.0))
    alpha = np.arccos(np.clip(cos_alpha, -1.0, 1.0))
    gamma = np.arctan2(y, x)

    phi1 = np.arctan2(-0.028, -0.11257)
    phi2 = np.arctan2(0.0052, -0.1349)

    theta_2 = gamma - alpha - phi1
    theta_3 = np.pi - beta - (np.pi / 2 + phi2 - phi1)

    theta_2 = (theta_2 + np.pi) % (2 * np.pi) - np.pi
    theta_3 = (theta_3 + np.pi) % (2 * np.pi) - np.pi

    joint_config["shoulder_lift"] = np.rad2deg(theta_2)
    joint_config["elbow_flex"] = np.rad2deg(theta_3)

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
