import time
import mujoco
import mujoco.viewer
from so101_mujoco_utils import set_initial_pose, send_position_command, move_to_pose, hold_position
from so101_inverse_kinematics import get_inverse_kinematics
import numpy as np

m = mujoco.MjModel.from_xml_path('model/scene.xml')
d = mujoco.MjData(m)

# Helper Function to show a cube at a given position and orientation
def show_cube(viewer, position, orientation, geom_num=0, halfwidth=0.013):
    mujoco.mjv_initGeom(
        viewer.user_scn.geoms[geom_num],
        type=mujoco.mjtGeom.mjGEOM_BOX, 
        size=[halfwidth, halfwidth, halfwidth],                 
        pos=position,                         
        mat=orientation.flatten(),              
        rgba=[1, 0, 0, 0.2]                           
    )
    viewer.user_scn.ngeom = 1
    viewer.sync()
    return
  
# Initial joint configuration at start of simulation
initial_config = {
    'shoulder_pan': 0.0,
    'shoulder_lift': 0.0,
    'elbow_flex': 0.00,
    'wrist_flex': 0.0,
    'wrist_roll': 0.0,
    'gripper': 0          
}
set_initial_pose(d, initial_config)
send_position_command(d, initial_config)

# Start simulation with mujoco viewer
with mujoco.viewer.launch_passive(m, d) as viewer:
  
  # Specify the desired position of the cube to be picked up
  desired_position = [0.2, 0.2, 0.014]

  # Add a cylinder as a site for visualization
  show_cube(viewer, desired_position, np.eye(3))
  
  # First send the robot to a higher position with the gripper open
  joint_configuration = get_inverse_kinematics(desired_position, viewer)
  move_to_pose(m, d, viewer, joint_configuration, 1.0)
  
  # Hold position for 10 seconds
  hold_position(m, d, viewer, 10.0)
