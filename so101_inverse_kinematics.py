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

    return joint_config