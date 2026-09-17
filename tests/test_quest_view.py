"""Viewer geometry and invalid/stale data behavior, without a headset or display."""

from dataclasses import replace
import importlib.util
from pathlib import Path
import sys
import unittest
from unittest.mock import Mock, patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
HAS_PLOTTING = all(importlib.util.find_spec(name) for name in ("numpy", "matplotlib"))
if HAS_PLOTTING:
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    import numpy as np
    from bimanual_teleop.visualization.quest import QuestPoseView, STALE_NS, pose_axes
    from bimanual_teleop.cli import view_quest as entry
    from bimanual_teleop.types import Health
    from tests.support.quest_protocol import sample


@unittest.skipUnless(HAS_PLOTTING, "install .[visualization] to test the optional viewer")
class QuestViewTests(unittest.TestCase):
    def setUp(self):
        self.view = QuestPoseView()
        self.sample = sample(refresh_hz=90)
        self.now = self.sample.header.received_monotonic_ns
        self.health = Health(True, self.now, "receiving")

    def tearDown(self):
        plt.close(self.view.figure)

    def test_quaternion_draws_positive_local_axes_without_another_basis_change(self):
        pose = replace(self.sample.payload.left, position_m=(1, 2, 3),
                       orientation_xyzw=(0, 0, 2**-0.5, 2**-0.5))
        axes = pose_axes(pose, length=1)
        np.testing.assert_allclose(axes[:, 0], [(1, 2, 3)] * 3)
        np.testing.assert_allclose(axes[:, 1], [(1, 3, 3), (0, 2, 3), (1, 2, 4)], atol=1e-12)

    def test_invalid_left_does_not_hide_right_when_aggregate_is_invalid(self):
        left = replace(self.sample.payload.left, location_flags=0, position_m=None,
                       orientation_xyzw=None, active=False)
        value = replace(self.sample, header=replace(self.sample.header, valid=False),
                        payload=replace(self.sample.payload, left=left))
        self.view.update(value, Health(False, self.now, "tracking unavailable"), self.now)
        self.assertFalse(self.view.artists["left"][0].get_visible())
        self.assertTrue(self.view.artists["right"][0].get_visible())
        self.view.figure.canvas.draw()

    def test_valid_untracked_pose_remains_visible_but_is_marked(self):
        left = replace(self.sample.payload.left, location_flags=7)
        value = replace(self.sample, payload=replace(self.sample.payload, left=left))
        self.view.update(value, self.health, self.now)
        self.assertTrue(self.view.artists["left"][0].get_visible())
        self.assertLess(self.view.artists["left"][0].get_alpha(), 1)
        self.assertIn("NOT tracked", self.view.readouts["left"].get_text())

    def test_stale_unfocused_and_missing_samples_hide_old_geometry(self):
        unfocused = replace(self.sample, payload=replace(self.sample.payload, session_state=4))
        for value, now in ((self.sample, self.now + STALE_NS + 1),
                           (unfocused, self.now), (None, self.now)):
            with self.subTest(sample=value, now=now):
                self.view.update(self.sample, self.health, self.now)
                self.view.update(value, self.health, now)
                self.assertTrue(all(not artist.get_visible()
                                    for artists in self.view.artists.values() for artist in artists))

    def test_camera_stays_fixed_until_fit_or_reference_change(self):
        self.view.update(self.sample, self.health, self.now)
        self.view.axes.set_xlim(-50, 50)
        self.view.axes.view_init(elev=12, azim=40)
        self.view.update(self.sample, self.health, self.now)
        self.assertEqual(self.view.axes.get_xlim(), (-50, 50))
        self.view._on_key(Mock(key="f"))
        self.assertNotEqual(self.view.axes.get_xlim(), (-50, 50))
        self.view.axes.set_xlim(-50, 50)
        header = replace(self.sample.header, ref=replace(self.sample.header.ref, epoch="new-origin"))
        self.view.update(replace(self.sample, header=header), self.health, self.now)
        self.assertNotEqual(self.view.axes.get_xlim(), (-50, 50))
        self.assertEqual((self.view.axes.elev, self.view.axes.azim), (12, 40))

    def test_closing_window_releases_source(self):
        source = Mock()
        with patch.object(entry, "QuestSource", return_value=source), patch.object(plt, "show"):
            self.assertEqual(entry.main([]), 0)
        source.start.assert_called_once()
        source.close.assert_called_once()


if __name__ == "__main__":
    unittest.main()
