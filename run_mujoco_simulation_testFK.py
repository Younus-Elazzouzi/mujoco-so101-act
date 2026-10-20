import time
import mujoco
import mujoco.viewer
from so101_mujoco_utils import set_initial_pose, send_position_command, move_to_pose, hold_position
from so101_forward_kinematics import get_forward_kinematics
import numpy as np

m = mujoco.MjModel.from_xml_path('model/scene.xml')
d = mujoco.MjData(m)

def show_frame(viewer, position, rotation, size=0.02):
    # Draw a 3-axis frame using cylinders and cones as arrows.
    # Each axis is aligned with the local z-direction of a geom, then
    # rotated into the object's frame so x, y, and z are shown.
    axis_length = 0.12
    shaft_radius = max(0.003, size * 0.2)
    cone_height = 0.03

    axis_rotations = {
        'x': np.array([[0.0, 0.0, 1.0],
                       [0.0, 1.0, 0.0],
                       [-1.0, 0.0, 0.0]], dtype=float),
        'y': np.array([[1.0, 0.0, 0.0],
                       [0.0, 0.0, 1.0],
                       [0.0, -1.0, 0.0]], dtype=float),
        'z': np.eye(3, dtype=float),
    }
    axis_colors = {'x': [1.0, 0.0, 0.0, 1.0],
                   'y': [0.0, 1.0, 0.0, 1.0],
                   'z': [0.0, 0.0, 1.0, 1.0]}

    geoms_used = 0
    for axis_name, local_axis_rot in axis_rotations.items():
        world_rot = rotation @ local_axis_rot
        axis_dir = world_rot[:, 2]

        # Shaft: cylinder extending from the base to the tip.
        shaft_half = axis_length / 2.0
        shaft_center = position + axis_dir * shaft_half
        mujoco.mjv_initGeom(
            viewer.user_scn.geoms[geoms_used],
            type=mujoco.mjtGeom.mjGEOM_CYLINDER,
            size=[shaft_radius, shaft_half, 0.0],
            pos=shaft_center,
            mat=world_rot.flatten(),
            rgba=axis_colors[axis_name]
        )
        geoms_used += 1

    viewer.user_scn.ngeom = geoms_used
    viewer.sync()
    return

def show_cylinder(viewer, position, rotation, radius=0.0245, halfheight=0.05, rgba=[1, 0, 0, 1]):
    # Add a cylinder aligned with z-axis
    mujoco.mjv_initGeom(
        viewer.user_scn.geoms[0],
        type=mujoco.mjtGeom.mjGEOM_CYLINDER,   # cylinder type
        size=[radius, halfheight, 0],                  # [radius, half-height, ignored]
        pos=position,                         # center position
        mat=rotation.flatten(),              # orientation matrix (identity = z-up)
        rgba=rgba                           # color
    )
    viewer.user_scn.ngeom = 1
    viewer.sync()
    return

test_configuration = {
    'shoulder_pan': -45.0,   # in radians for mujoco! 
    'shoulder_lift': 45.0,
    'elbow_flex': -45.00,
    'wrist_flex': 90.0,
    'wrist_roll': 0.0,
    'gripper': 10          
}

set_initial_pose(d, test_configuration)
send_position_command(d, test_configuration)

object_position, object_orientation = get_forward_kinematics(test_configuration)

with mujoco.viewer.launch_passive(m, d) as viewer:
  # Close the viewer automatically after 30 wall-seconds.
  start = time.time()
  
  # Add a cylinder as a site for visualization
  show_frame(viewer, object_position, object_orientation)
  
  # Hold Starting Position for 2 seconds
  hold_position(m, d, viewer, 20.0)
