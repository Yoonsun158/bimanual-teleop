"""Live controller coordinate frames in the adapter's LOCAL forward/left/up axes."""

from __future__ import annotations

import time
import textwrap
from typing import Any

from bimanual_teleop.devices.quest.adapter import QuestFrame, QuestPose
from bimanual_teleop.types import Health, Sample

AXIS_COLORS = ("#dd4444", "#27964b", "#3478d4")
STALE_NS = 250_000_000  # Host receive silence, not a source-age/latency estimate.


def pose_axes(pose: QuestPose, length: float = 0.18) -> Any:
    """Return three positive-axis segments; xyzw is already in project coordinates."""
    import numpy as np

    position = np.asarray(pose.position_m)
    q = np.asarray(pose.orientation_xyzw)
    x, y, z, w = q / np.linalg.norm(q)
    rotation = np.array([
        [1 - 2*(y*y + z*z), 2*(x*y - z*w), 2*(x*z + y*w)],
        [2*(x*y + z*w), 1 - 2*(x*x + z*z), 2*(y*z - x*w)],
        [2*(x*z - y*w), 2*(y*z + x*w), 1 - 2*(x*x + y*y)],
    ])
    return np.stack((np.tile(position, (3, 1)), position + length * rotation.T), axis=1)


class QuestPoseView:
    """Update existing artists on the GUI thread; acquisition remains in QuestSource."""

    def __init__(self) -> None:
        import matplotlib.pyplot as plt
        from mpl_toolkits.mplot3d.art3d import Line3DCollection

        self.figure = plt.figure(figsize=(9, 8))
        self.figure.canvas.manager.set_window_title("Quest controller poses")
        self.axes = self.figure.add_subplot(projection="3d")
        self.figure.subplots_adjust(left=0.06, right=0.94, bottom=0.20, top=0.84)
        self.axes.set(xlabel="X forward (m)", ylabel="Y left (m)", zlabel="Z up (m)")
        self.axes.set_box_aspect((1, 1, 1))
        self.axes.view_init(elev=25, azim=-135)
        self.axes.set(xlim=(-1, 1), ylim=(-1, 1), zlim=(-1, 1))
        self.axes.plot([0], [0], [0], marker="+", color="black", markersize=9)
        self.title = self.figure.suptitle("Quest controllers | waiting for data", fontsize=15)
        for i, (axis, color) in enumerate(zip("XYZ", AXIS_COLORS)):
            self.figure.text(0.35 + i * 0.11, 0.925, f"{axis} axis", color=color, weight="bold")
        self.status = self.figure.text(0.06, 0.115, "", fontsize=9, va="top")
        self.figure.text(0.06, 0.035,
                         "Drag: rotate | Right-drag: zoom | F: fit controllers | Close: stop",
                         fontsize=9, color="#555555")
        self.artists = {}
        self.readouts = {}
        for i, (side, color) in enumerate((("left", "#8d49b8"), ("right", "#d77b19"))):
            axes = Line3DCollection([], colors=AXIS_COLORS, linewidths=3)
            self.axes.add_collection3d(axes, autolim=False)
            marker, = self.axes.plot([], [], [], marker="o", color=color, markersize=8)
            label = self.axes.text(0, 0, 0, side.upper(), color=color, weight="bold")
            self.artists[side] = (axes, marker, label)
            for artist in self.artists[side]:
                artist.set_visible(False)
            self.readouts[side] = self.figure.text(
                0.06, 0.175 - i * 0.028, f"{side.upper()}: waiting", color=color,
                fontsize=10, family="monospace")
        self._positions = []
        self._epoch = None
        self._needs_fit = True
        self.figure.canvas.mpl_connect("key_press_event", self._on_key)

    def _on_key(self, event) -> None:
        if event.key == "f":
            self.fit()
            self.figure.canvas.draw_idle()

    def fit(self) -> None:
        """Change the view bounds only; preserve the source frame and camera angle."""
        import numpy as np

        if not self._positions:
            return
        points = np.asarray(self._positions)
        center = (points.min(axis=0) + points.max(axis=0)) / 2
        radius = max(0.6, float(np.ptp(points, axis=0).max()) / 2 + 0.3)
        for setter, value in zip((self.axes.set_xlim, self.axes.set_ylim, self.axes.set_zlim), center):
            setter(value - radius, value + radius)
        self._needs_fit = False

    def update(self, sample: Sample[QuestFrame] | None, health: Health,
               now_ns: int | None = None) -> None:
        import numpy as np

        now_ns = time.monotonic_ns() if now_ns is None else now_ns
        frame = sample.payload if sample else None
        blocked = ""
        if sample is None:
            blocked = "waiting"
        elif now_ns - sample.header.received_monotonic_ns > STALE_NS:
            blocked = "stale: no new frame for 250 ms"
        elif frame.session_state != 5:
            blocked = "XR session not focused"
        if sample and sample.header.ref.epoch != self._epoch:
            self._epoch = sample.header.ref.epoch
            self._needs_fit = True
        self._positions = []
        for side, artists in self.artists.items():
            pose = getattr(frame, side) if frame else None
            visible = (not blocked and pose is not None and pose.valid and pose.active is True)
            for artist in artists:
                artist.set_visible(bool(visible))
            if visible:
                axes, marker, label = artists
                position = pose.position_m
                axes.set_segments(pose_axes(pose))
                axes.set_alpha(1.0 if pose.tracked else 0.3)
                axes.set_linestyle("solid" if pose.tracked else "dashed")
                marker.set_data_3d(*([value] for value in position))
                label.set_position_3d(np.asarray(position) + (0, 0, 0.06))
                state = "tracked" if pose.tracked else "valid, NOT tracked"
                text = f"{state} | p=({position[0]:+.3f}, {position[1]:+.3f}, {position[2]:+.3f}) m"
                self._positions.append(position)
            else:
                text = blocked or "inactive / invalid pose"
            self.readouts[side].set_text(f"{side.upper():5}: {text}")
        if self._needs_fit:
            self.fit()
        info = (f"LOCAL {frame.origin} | frame {frame.sequence} | XR {frame.refresh_hz:g} Hz"
                if frame else "waiting for data")
        self.title.set_text(f"Quest controllers | {info}")
        detail = blocked or health.detail or "receiving"
        self.status.set_text(textwrap.fill(f"Source: {detail}", width=110))
        self.status.set_color("#555555" if health.ready and not blocked else "#b45309")
        self.figure.canvas.draw_idle()
