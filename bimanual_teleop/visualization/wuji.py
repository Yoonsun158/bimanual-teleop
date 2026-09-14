"""Read-only hand joints and calibrated tactile contact/pressure display."""

from __future__ import annotations

import time

from bimanual_teleop.devices.wuji.adapter import WujiTactileFrame
from bimanual_teleop.types import HandSkeleton

STALE_NS = 500_000_000
CONTACT_SYNC_NS = 100_000_000
FINGER_PATHS = ((0, 1, 2, 3, 4), (0, 5, 6, 7, 8), (0, 9, 10, 11, 12),
                (0, 13, 14, 15, 16), (0, 17, 18, 19, 20))
PALM_PATH = (0, 1, 5, 9, 13, 17, 0)
FINGER_COLORS = ("#d97706", "#0284c7", "#059669", "#7c3aed", "#db2777")


def live_payload(sample, expected_type, now_ns):
    if sample is None:
        return None, "waiting"
    if not sample.header.valid or not isinstance(sample.payload, expected_type):
        return None, "INVALID frame"
    age = now_ns - sample.header.received_monotonic_ns
    if age < 0 or age > STALE_NS:
        return None, "stale frame"
    return sample.payload, "live"


class WujiGloveView:
    def __init__(self, side):
        import matplotlib.pyplot as plt
        import numpy as np
        from matplotlib.collections import LineCollection
        from matplotlib.colors import LinearSegmentedColormap, Normalize

        self.side = side
        self.figure = plt.figure(figsize=(12, 7.5), facecolor="#f8fafc")
        self.figure.canvas.manager.set_window_title(f"Wuji Glove · {side}")
        grid = self.figure.add_gridspec(1, 2, width_ratios=(1, 1.15),
                                        left=.03, right=.94, bottom=.14, top=.74, wspace=.12)
        self.skeleton_ax = self.figure.add_subplot(grid[0, 0], projection="3d", facecolor="#f8fafc")
        self.skeleton_ax.set_proj_type("ortho")
        self.skeleton_ax.view_init(elev=12, azim=-80)
        self.skeleton_ax.set(xlim=(-125, 125), ylim=(-100, 100), zlim=(-25, 245))
        self.skeleton_ax.set_box_aspect((250, 200, 270), zoom=1.5)
        self.skeleton_ax.set_axis_off()
        self.skeleton_lines = [self.skeleton_ax.plot([], [], [], color=color,
            lw=3, marker="o", ms=5, markeredgecolor="white", markeredgewidth=.8)[0]
            for color in FINGER_COLORS]
        self.palm_line, = self.skeleton_ax.plot([], [], [], color="#94a3b8", lw=2)
        self.wrist_label = self.skeleton_ax.text(0, 0, -18, "WRIST", ha="center", color="#64748b", fontsize=9)
        self.wrist_label.set_visible(False)

        self.tactile_ax = self.figure.add_subplot(grid[0, 1], facecolor="white")
        palette = LinearSegmentedColormap.from_list("contact_pressure", [
            (0., "#e2e8f0"), (.01, "#93c5fd"), (.2, "#22d3ee"),
            (.45, "#facc15"), (.7, "#f97316"), (1., "#dc2626")])
        palette.set_bad("white")
        self.heat = self.tactile_ax.imshow(np.ma.masked_all((24, 31)),
            interpolation="nearest", aspect="equal", origin="upper", cmap=palette,
            norm=Normalize(vmin=0., vmax=1., clip=True))
        self.contact_outline = LineCollection([], colors="#0f172a", linewidths=1.)
        self.tactile_ax.add_collection(self.contact_outline)
        self.tactile_ax.set(xlabel="sensor column", ylabel="sensor row")
        self.tactile_ax.tick_params(labelsize=8, colors="#64748b")
        for spine in self.tactile_ax.spines.values():
            spine.set_color("#cbd5e1")
        colorbar = self.figure.colorbar(self.heat, ax=self.tactile_ax, fraction=.045, pad=.04)
        colorbar.set_label("Relative pressure (0–1)", color="#475569", fontsize=10)
        colorbar.set_ticks([0, .2, .4, .6, .8, 1])
        colorbar.outline.set_visible(False)
        colorbar.ax.tick_params(labelsize=8)

        self.title = self.figure.suptitle(f"Wuji Glove · {side}", x=.05, y=.97,
                                          ha="left", fontsize=20, color="#0f172a", weight="bold")
        self.figure.text(.05, .905, "HAND JOINTS", fontsize=11, weight="bold", color="#475569")
        self.figure.text(.05, .862, "Fingertips up · wrist down", fontsize=10, color="#64748b")
        self.figure.text(.05, .07, "Drag to rotate the hand · geometry in wrist coordinates",
                          fontsize=9, color="#64748b")
        self.figure.text(.52, .905, "TACTILE CONTACT & PRESSURE", fontsize=11,
                          weight="bold", color="#475569")
        self.contact_label = self.figure.text(.52, .847, "WAITING FOR TACTILE", fontsize=16,
                                              weight="bold", color="#64748b")
        self.pressure_label = self.figure.text(.52, .798, "Peak contact pressure —", fontsize=11,
                                               color="#475569")
        self.tactile_legend = self.figure.text(.52, .07,
            "Black outline = contact · gray = no contact · white = no sensor",
            fontsize=9, color="#64748b")
        self.figure.text(.52, .036, "Pressure is relative sensor output, not force in newtons.",
                          fontsize=9, color="#64748b")
        self.labels = [self.figure.text(x, .765, "waiting", fontsize=10, color="#64748b")
                       for x in (.05, .52)]

    def update(self, source, now_ns=None, session=None):
        import numpy as np

        now_ns = time.monotonic_ns() if now_ns is None else now_ns
        problem = getattr(source, "fault", None)
        if session is not None:
            health = session.health()
            if not health.ready:
                problem = health.detail or "SDK user changed"
        samples = [source.get_latest_stream(name) for name in ("skeleton", "tactile", "contact")]
        if problem:
            skeleton = tactile = contact = None
            skeleton_state = tactile_state = contact_state = f"INVALID: {problem}"
        else:
            skeleton, skeleton_state = live_payload(samples[0], HandSkeleton, now_ns)
            tactile, tactile_state = live_payload(samples[1], WujiTactileFrame, now_ns)
            contact, contact_state = live_payload(samples[2], WujiTactileFrame, now_ns)

        # A display-only 180° rotation about X: SDK +Z points to the elbow.
        # Keep the measured skeleton and all retargeting coordinates untouched.
        points = np.asarray(skeleton.positions_m) * (1000., -1000., -1000.) if skeleton else None
        for artist, path in zip((*self.skeleton_lines, self.palm_line), (*FINGER_PATHS, PALM_PATH)):
            artist.set_data_3d(points[list(path)].T if skeleton else np.empty((3, 0)))
        self.wrist_label.set_visible(skeleton is not None)

        shape = (tactile.rows, tactile.columns) if tactile else self.heat.get_array().shape
        pressure = (np.asarray(tactile.values, dtype=float).reshape(shape)
                    if tactile else np.full(shape, np.nan))
        active = np.isfinite(pressure) & (pressure >= 0)
        contacts = np.zeros(shape, dtype=bool)
        known_contact = False
        if tactile:
            if not source.metadata.get("tactile_contact_model_present", False):
                contact_state = "model missing; calibrate tactile"
            elif contact is not None:
                binary = np.asarray(contact.values).reshape(contact.rows, contact.columns)
                if binary.shape != shape or not np.array_equal(binary >= 0, active):
                    contact_state = "layout mismatch"
                elif abs(samples[1].header.received_monotonic_ns
                         - samples[2].header.received_monotonic_ns) > CONTACT_SYNC_NS:
                    contact_state = "waiting for synchronized frames"
                else:
                    known_contact = True
                    contacts = (binary == 1) & active

        if tactile is None or not active.any():
            self.contact_label.set_text("TACTILE UNAVAILABLE")
            self.contact_label.set_color("#64748b")
            self.pressure_label.set_text("Peak contact pressure —")
        elif known_contact:
            count = int(contacts.sum())
            self.contact_label.set_text(f"{'CONTACT' if count else 'NO CONTACT'}  ·  {count} / {active.sum()} points")
            self.contact_label.set_color("#c2410c" if count else "#15803d")
            peak = float(pressure[contacts].max()) if count else 0.
            self.pressure_label.set_text(f"Peak contact pressure  {peak:.3f} / 1.000")
        else:
            self.contact_label.set_text("CONTACT UNKNOWN")
            self.contact_label.set_color("#b45309")
            self.pressure_label.set_text(f"Pressure only · peak {pressure[active].max():.3f} / 1.000")

        shown = np.where(contacts, pressure, 0.) if known_contact else pressure
        self.heat.set_data(np.ma.array(shown, mask=~active))
        self.tactile_ax.set_xlim(-.5, shape[1] - .5)
        self.tactile_ax.set_ylim(shape[0] - .5, -.5)
        boxes = [[(col-.5, row-.5), (col+.5, row-.5), (col+.5, row+.5),
                  (col-.5, row+.5), (col-.5, row-.5)] for row, col in np.argwhere(contacts)]
        self.contact_outline.set_segments(boxes)
        self.tactile_legend.set_text(
            "Black outline = contact · gray = no contact · white = no sensor" if known_contact
            else "Colors = pressure only · white = no sensor or unavailable")
        self.labels[0].set_text(f"skeleton: {skeleton_state}")
        self.labels[1].set_text(f"tactile: {tactile_state} · contact: {contact_state}")
        self.title.set_text(f"Wuji Glove · {self.side.capitalize()} · "
                            f"{source.metadata.get('sdk_user_name') or 'default'}"
                            + (f" · INVALID: {problem}" if problem else ""))
        self.figure.canvas.draw_idle()
