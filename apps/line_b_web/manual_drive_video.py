"""Record the validated Line-B locomotion sequence in MuJoCo.

The get-up and zero-speed handoff run as an unrecorded pre-roll.  The video then
shows forward, backward, left turn, forward-left and backward commands using the
original real-robot locomotion profile (speed_scale=0.5).
"""

from __future__ import annotations

import argparse
import subprocess
from pathlib import Path

import cv2
import mujoco
import numpy as np

from apps import ref
from apps.line_b_web.control import LineBControl
from apps.line_b_web.machine import HandoffConfig, LYING, STAND_HOLD, WALK, LineBMachine
from apps.teleop.sim import Display, draw_text
from runtime import MuJoCoBackend, Package

PROJECT = Path(__file__).resolve().parents[2]
DEFAULT_OUT = PROJECT / "deliverables" / "q1_line_b_mujoco_original_params.mp4"
FPS = 25
HEIGHT = 720
WIDTH = 960


def run_until(machine: LineBMachine, control: LineBControl, target: str, timeout_s: float) -> None:
    for _ in range(int(round(timeout_s / machine.dt))):
        control.set_status({"mode": machine.mode})
        pad, edges = control.sample()
        machine.tick(pad, edges)
        if machine.mode == target:
            return
    raise RuntimeError(f"MuJoCo pre-roll did not reach {target}; final mode={machine.mode}")


def add_trail(scene: mujoco.MjvScene, points: list[np.ndarray]) -> None:
    """Draw recent pelvis positions as small cyan dots so translation is visible."""
    for point in points[-240:]:
        if scene.ngeom >= scene.maxgeom:
            return
        geom = scene.geoms[scene.ngeom]
        mujoco.mjv_initGeom(
            geom,
            mujoco.mjtGeom.mjGEOM_SPHERE,
            np.array([0.014, 0.014, 0.014]),
            np.array([point[0], point[1], 0.018]),
            np.eye(3).reshape(-1),
            np.array([0.05, 0.75, 0.95, 0.85], dtype=np.float32),
        )
        scene.ngeom += 1


def overlay(frame: np.ndarray, action: str, display: Display) -> np.ndarray:
    rows = [("ACTION", action, ""), ("profile", "scale 0.5", "50 Hz")] + display.rows()
    frame = draw_text(frame, rows)
    cv2.putText(
        frame,
        "cyan: travelled path   green: command velocity   orange: measured velocity",
        (18, HEIGHT - 22),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.55,
        (235, 235, 235),
        1,
        cv2.LINE_AA,
    )
    return frame


class VideoWriter:
    def __init__(self, path: Path):
        path.parent.mkdir(parents=True, exist_ok=True)
        self.path = path
        self.proc = subprocess.Popen(
            [
                "ffmpeg", "-loglevel", "error", "-y",
                "-f", "rawvideo", "-pix_fmt", "rgb24",
                "-s", f"{WIDTH}x{HEIGHT}", "-r", str(FPS), "-i", "-",
                "-an", "-c:v", "libx264", "-preset", "medium", "-crf", "20",
                "-pix_fmt", "yuv420p", "-movflags", "+faststart", str(path),
            ],
            stdin=subprocess.PIPE,
        )

    def append(self, frame: np.ndarray) -> None:
        if self.proc.stdin is None:
            raise RuntimeError("ffmpeg stdin is unavailable")
        self.proc.stdin.write(np.ascontiguousarray(frame, dtype=np.uint8).tobytes())

    def close(self) -> None:
        if self.proc.stdin is not None:
            self.proc.stdin.close()
        code = self.proc.wait()
        if code:
            raise RuntimeError(f"ffmpeg exited with {code}")


def main() -> None:
    parser = argparse.ArgumentParser(description="Record Line-B MuJoCo manual-drive preview")
    parser.add_argument("--out", type=Path, default=DEFAULT_OUT)
    args = parser.parse_args()

    base = Package.load(PROJECT / "weights/dual_v3b")
    skill = Package.load(PROJECT / "weights/getup_vpd_ft")
    dance = Package.load(PROJECT / "weights/q1_smooth_dance")
    robot = MuJoCoBackend(base)
    machine = LineBMachine(
        base,
        skill,
        robot,
        ref.find("getup_facedown"),
        ref.find("liedown_facedown"),
        dance,
        dance.root / "motion.npz",
        speed_scale=0.5,
        experimental_handoff=HandoffConfig(),
        dance_enabled=False,
    )
    control = LineBControl(
        drive_timeout=1000.0,
        heartbeat_timeout=1000.0,
        mobility_enabled=True,
        dance_enabled=False,
    )
    control.session(True)
    control.set_status({"mode": LYING})
    control.action("getup")
    run_until(machine, control, STAND_HOLD, 12.0)
    control.action("walk")
    run_until(machine, control, WALK, 16.0)

    renderer = mujoco.Renderer(robot.model, height=HEIGHT, width=WIDTH)
    camera = mujoco.MjvCamera()
    camera.type = mujoco.mjtCamera.mjCAMERA_TRACKING
    camera.trackbodyid = robot.pelvis
    camera.distance, camera.elevation, camera.azimuth = 2.15, -17.0, 145.0
    display = Display(machine)
    writer = VideoWriter(args.out)
    trail: list[np.ndarray] = []
    frame_stride = max(1, int(round(1.0 / (FPS * machine.dt))))
    segments = [
        ("STAND", 0.0, 0.0, 1.0),
        ("FORWARD  +0.30 m/s", 1.0, 0.0, 2.5),
        ("STOP", 0.0, 0.0, 1.0),
        ("BACKWARD  -0.20 m/s", -1.0, 0.0, 2.5),
        ("STOP", 0.0, 0.0, 1.0),
        ("TURN LEFT  +0.50 rad/s", 0.0, 1.0, 2.0),
        ("STOP", 0.0, 0.0, 1.0),
        ("FORWARD + LEFT", 1.0, 1.0, 2.5),
        ("STOP", 0.0, 0.0, 1.0),
        ("BACKWARD  -0.20 m/s", -1.0, 0.0, 2.5),
        ("FINAL STOP", 0.0, 0.0, 1.5),
    ]

    step = 0
    try:
        for label, ly, rx, duration_s in segments:
            for _ in range(int(round(duration_s / machine.dt))):
                control.set_status({"mode": machine.mode})
                control.drive(ly=ly, rx=rx)
                pad, edges = control.sample()
                machine.tick(pad, edges)
                display.update()
                pelvis = robot.data.xpos[robot.pelvis].copy()
                if not trail or np.linalg.norm(pelvis[:2] - trail[-1][:2]) >= 0.035:
                    trail.append(pelvis)
                if step % frame_stride == 0:
                    renderer.update_scene(robot.data, camera)
                    add_trail(renderer.scene, trail)
                    display.add_arrows(renderer.scene)
                    writer.append(overlay(renderer.render(), label, display))
                step += 1
        control.drive(ly=0.0, rx=0.0)
    finally:
        writer.close()
        renderer.close()
        robot.damp()

    print(f"video: {args.out}")
    print(f"final mode: {machine.mode}; vx={machine.cmd[0]:+.3f}; wz={machine.cmd[2]:+.3f}")


if __name__ == "__main__":
    main()
