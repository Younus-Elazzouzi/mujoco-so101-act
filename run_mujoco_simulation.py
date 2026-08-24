import argparse
import math
import random
import time

import mujoco
import mujoco.viewer
import numpy as np
from so101_mujoco_utils import set_initial_pose, move_to_pose, hold_position
from mujoco_video_recorder import MujocoVideoRecorder


parser = argparse.ArgumentParser()
parser.add_argument('--record', action='store_true', help='Save the run as an MP4 video.')
parser.add_argument('--output', default='recordings/mujoco_run.mp4', help='MP4 path used with --record.')
parser.add_argument('--fps', type=int, default=30, help='Recording frame rate.')
args = parser.parse_args()

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

recorder = MujocoVideoRecorder(m, args.output, fps=args.fps) if args.record else None

def show_cube(viewer, position, rotation, halfsize=0.01, rgba=[1, 0, 0, 1]):
    # Add a cube aligned with z-axis
    mujoco.mjv_initGeom(
        viewer.user_scn.geoms[0],
        type=mujoco.mjtGeom.mjGEOM_BOX,   # box type
        size=[halfsize, halfsize, halfsize],                  # [half-length, half-width, half-height]
        pos=position,                         # center position
        mat=rotation.flatten(),              # orientation matrix (identity = z-up)
        rgba=rgba,                           # color
    )
    viewer.user_scn.ngeom = 1
    viewer.sync()
    return

def show_cylinder(viewer, position, rotation, radius=0.025, halfheight=0.002, rgba=[0, 1, 0, 1]):
    mujoco.mjv_initGeom(
        viewer.user_scn.geoms[1],
        type=mujoco.mjtGeom.mjGEOM_CYLINDER,
        size=[radius, halfheight, 0],
        pos=position,
        mat=rotation.flatten(),
        rgba=rgba,
    )
    viewer.user_scn.ngeom = 2
    viewer.sync()
    return

def random_cube_position(tx, ty, x_range=(0.1, 0.4), y_range=(-0.4, 0.4), min_distance=0.075, max_distance=0.35):
    
    while True:
        x = random.uniform(*x_range)
        y = random.uniform(*y_range)

        dist_to_arm = math.hypot(x, y)

        dist_to_target = math.hypot(x - tx, y - ty)

        # Valid only if safely away from both
        if max_distance >= dist_to_arm >= 2*min_distance and dist_to_target >= min_distance:
            return x, y

target_x, target_y = (0.2, 0)
cube_x, cube_y = random_cube_position(target_x, target_y)
cube_halfsize = 0.0125
cylinder_radius = 0.025
cylinder_halfheight = 0.002

try:
  with mujoco.viewer.launch_passive(m, d) as viewer:
    capture_frame = recorder.capture if recorder else None

    show_cube(
      viewer,
      position=[cube_x, cube_y, cube_halfsize],
      halfsize=cube_halfsize,
      rotation=np.eye(3)
    )

    show_cylinder(
      viewer,
      position=[target_x, target_y, cylinder_halfheight],
      radius=cylinder_radius,
      halfheight=cylinder_halfheight,
      rotation=np.eye(3)
    )

    # Go to desired position
    move_to_pose(m, d, viewer, desired_position, 2.0, capture_frame)

    # Hold Position
    hold_position(m, d, viewer, 2.0, capture_frame)

    # Return to starting
    move_to_pose(m, d, viewer, starting_position, 2.0, capture_frame)
finally:
  if recorder:
    recorder.close()
    print(f'Recording saved to {args.output}')

  # force quit everything
  import os, signal
  time.sleep(2)  # Give the viewer a moment to close gracefully
  os.kill(os.getpid(), signal.SIGTERM)

