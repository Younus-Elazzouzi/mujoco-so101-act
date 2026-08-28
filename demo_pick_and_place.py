import time
import mujoco
import mujoco.viewer
from so101_mujoco_utils import set_initial_pose, send_position_command, move_to_pose, hold_position
from so101_forward_kinematics import get_forward_kinematics
import numpy as np

m = mujoco.MjModel.from_xml_path('model/scene.xml')
d = mujoco.MjData(m)

def show_cubes(viewer, starting_config, final_config, halfwidth=0.013):
    # Use forward kinematics
    starting_object_position, starting_object_orientation = get_forward_kinematics(starting_config)
    final_object_position, final_object_orientation = get_forward_kinematics(final_config)
  
    # Add starting cube
    mujoco.mjv_initGeom(
        viewer.user_scn.geoms[0],
        type=mujoco.mjtGeom.mjGEOM_BOX, 
        size=[halfwidth, halfwidth, halfwidth],                 
        pos=starting_object_position,                         
        mat=starting_object_orientation.flatten(),              
        rgba=[1, 0, 0, 0.2]                           
    )
    # Add final cube
    mujoco.mjv_initGeom(
        viewer.user_scn.geoms[1],
        type=mujoco.mjtGeom.mjGEOM_BOX, 
        size=[halfwidth, halfwidth, halfwidth],                 
        pos=final_object_position,                         
        mat=final_object_orientation.flatten(),              
        rgba=[0, 1, 0, 0.2]                           
    )
    viewer.user_scn.ngeom = 2
    viewer.sync()
    return

starting_configuration = {
    'shoulder_pan': -45.0,   # in radians for mujoco! 
    'shoulder_lift': 45.0,
    'elbow_flex': -45.00,
    'wrist_flex': 90.0,
    'wrist_roll': 0.0,
    'gripper': 50
}
final_configuration = {
    'shoulder_pan': 45.0,   # in radians for mujoco! 
    'shoulder_lift': 45.0,
    'elbow_flex': -45.00,
    'wrist_flex': 90.0,
    'wrist_roll': 0.0,
    'gripper': 50       
}
starting_configuration_closed = {
    'shoulder_pan': -45.0,   # in radians for mujoco! 
    'shoulder_lift': 45.0,
    'elbow_flex': -45.00,
    'wrist_flex': 90.0,
    'wrist_roll': 0.0,
    'gripper': 5
}
final_configuration_closed = {
    'shoulder_pan': 45.0,   # in radians for mujoco! 
    'shoulder_lift': 45.0,
    'elbow_flex': -45.00,
    'wrist_flex': 90.0,
    'wrist_roll': 0.0,
    'gripper': 5    
}
lift_up_configuration_closed = {
    'shoulder_pan': -45.0,   # in radians for mujoco! 
    'shoulder_lift': 0.0,
    'elbow_flex': 0.0,
    'wrist_flex': 90.0,
    'wrist_roll': 0.0,
    'gripper': 5    
}
rotate_configuration_closed = {
    'shoulder_pan': 45.0,   # in radians for mujoco! 
    'shoulder_lift': 0.0,
    'elbow_flex': 0.0,
    'wrist_flex': 90.0,
    'wrist_roll': 0.0,
    'gripper': 5    
}


set_initial_pose(d, starting_configuration)
send_position_command(d, starting_configuration)

with mujoco.viewer.launch_passive(m, d) as viewer:
  # Close the viewer automatically after 30 wall-seconds.
  start = time.time()
  
  # Add cubes as a site for visualization
  show_cubes(viewer, starting_configuration, final_configuration)
  
  # Start pick and place
  move_to_pose(m, d, viewer, starting_configuration_closed, 2.0)

  hold_position(m, d, viewer, 2.0)

  move_to_pose(m, d, viewer, lift_up_configuration_closed, 2.0)

  hold_position(m, d, viewer, 2.0)

  move_to_pose(m, d, viewer, rotate_configuration_closed, 3.0)

  hold_position(m, d, viewer, 2.0)

  move_to_pose(m, d, viewer, final_configuration_closed, 2.0)

  hold_position(m, d, viewer, 2.0)

  move_to_pose(m, d, viewer, final_configuration, 2.0)

  hold_position(m, d, viewer, 2.0)
