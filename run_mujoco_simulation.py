import mujoco
import mujoco.viewer
from so101_mujoco_utils import set_initial_pose, move_to_pose, hold_position

m = mujoco.MjModel.from_xml_path('model/scene.xml')
d = mujoco.MjData(m)

starting_position = {
    'shoulder_pan': 2.51e-14 * 180.0 / 3.14159,
    'shoulder_lift': -1.75 * 180.0 / 3.14159,
    'elbow_flex': 1.59 * 180.0 / 3.14159,
    'wrist_flex': 1.1 * 180.0 / 3.14159,
    'wrist_roll': 3.76e-08 * 180.0 / 3.14159,
    'gripper': -3.54e-06 * 100 / 3.14159
}

desired_position = {
    'shoulder_pan': 0.0,   # in degrees
    'shoulder_lift': 0.0,
    'elbow_flex': 0.0,
    'wrist_flex': 0.0,
    'wrist_roll': 0.0,
    'gripper': 0.0           # 0-100 range
}

set_initial_pose(d, starting_position)

with mujoco.viewer.launch_passive(m, d) as viewer:

  # Go to desired position
  move_to_pose(m, d, viewer, desired_position, 2.0)
  
  # Hold Position
  hold_position(m, d, viewer, 2.0)
  
  # Return to starting
  move_to_pose(m, d, viewer, starting_position, 2.0)
