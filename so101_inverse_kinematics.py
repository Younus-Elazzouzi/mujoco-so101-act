import numpy as np

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