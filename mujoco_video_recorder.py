from pathlib import Path
import time

import imageio
import mujoco


class MujocoVideoRecorder:
    def __init__(self, model, output_path, fps=30, width=640, height=480):
        self.model = model
        self.output_path = Path(output_path)
        self.output_path.parent.mkdir(parents=True, exist_ok=True)
        self.fps = fps
        self.width = width
        self.height = height
        self.frame_interval = 1 / fps
        self.next_frame_time = None
        self.frames = []

    def capture(self, data):
        """Snapshot the pose at a fixed wall-clock frame rate.

        Rendering is deliberately deferred until the simulation is finished:
        off-screen rendering is expensive enough to change this script's
        wall-time-controlled motion otherwise.
        """
        now = time.monotonic()
        if self.next_frame_time is None:
            self.next_frame_time = now
        if now < self.next_frame_time:
            return

        # Add repeated poses if an occasional iteration takes longer than one
        # video frame, preserving the real-time duration of the recording.
        pose = data.qpos.copy()
        velocity = data.qvel.copy()
        while now >= self.next_frame_time:
            self.frames.append((pose, velocity))
            self.next_frame_time += self.frame_interval

    def close(self):
        if not self.frames:
            return

        render_data = mujoco.MjData(self.model)
        renderer = mujoco.Renderer(self.model, width=self.width, height=self.height)
        writer = imageio.get_writer(self.output_path, fps=self.fps)
        try:
            for pose, velocity in self.frames:
                render_data.qpos[:] = pose
                render_data.qvel[:] = velocity
                mujoco.mj_forward(self.model, render_data)
                renderer.update_scene(render_data)
                writer.append_data(renderer.render())
        finally:
            writer.close()
            renderer.close()
