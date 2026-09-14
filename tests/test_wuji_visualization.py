"""Wuji skeleton orientation, contact semantics, and pressure color stability."""

import os
from types import SimpleNamespace as NS
import unittest

os.environ.setdefault("MPLBACKEND", "Agg")

from bimanual_teleop.devices.wuji.adapter import SKELETON_NAMES, WujiTactileFrame
from bimanual_teleop.visualization.wuji import WujiGloveView, STALE_NS
from bimanual_teleop.types import HandSkeleton, Sample, SampleHeader, SampleRef


def frame(stream, payload, valid=True, now_ns=100):
    return Sample(SampleHeader(SampleRef(stream, "epoch", 0), now_ns, valid), payload)


def skeleton(side):
    points = [(0., 0., 0.)]
    for finger in range(5):
        points.extend(((2 - finger) * .025, 0., -.03 - joint * .035) for joint in range(4))
    return HandSkeleton(f"{side[0]}_wrist", SKELETON_NAMES, tuple(points), (.9,) * 21, ())


class ViewTests(unittest.TestCase):
    def view(self, side="left"):
        import matplotlib.pyplot as plt
        view = WujiGloveView(side)
        self.addCleanup(plt.close, view.figure)
        return view

    def source(self, data, model=True):
        return NS(get_latest_stream=lambda stream: data.get(stream), fault=None,
                  metadata={"sdk_user_name": "yuchen", "tactile_contact_model_present": model})

    def test_fingertips_project_above_wrist_without_changing_measured_geometry(self):
        import numpy as np
        from mpl_toolkits.mplot3d import proj3d

        for side in ("left", "right"):
            with self.subTest(side=side):
                view = self.view(side)
                hand = skeleton(side)
                original = hand.positions_m
                view.update(self.source({"skeleton": frame("skeleton", hand)}), now_ns=100)
                self.assertEqual(len(view.figure.axes), 3)  # skeleton, tactile, pressure legend
                self.assertFalse(hasattr(view, "bars"))
                for line in view.skeleton_lines:
                    x, y, z = line.get_data_3d()
                    self.assertGreater(z[-1], z[0])
                    _, projected_y, _ = proj3d.proj_transform(x, y, z, view.skeleton_ax.get_proj())
                    self.assertGreater(projected_y[-1], projected_y[0])
                drawn = np.stack(view.skeleton_lines[0].get_data_3d(), axis=1)
                np.testing.assert_allclose(np.linalg.norm(np.diff(drawn, axis=0), axis=1),
                    np.linalg.norm(np.diff(np.asarray(original)[:5], axis=0), axis=1) * 1000)
                self.assertEqual(hand.positions_m, original)
                self.assertIn("yuchen", view.title.get_text())
                self.assertIn("live", view.labels[0].get_text())

    def test_contact_regions_and_fixed_pressure_scale_for_both_layouts(self):
        for columns in (31, 32):
            with self.subTest(columns=columns):
                view = self.view()
                pressure, contact = [.7] * (24 * columns), [0.] * (24 * columns)
                pressure[0] = contact[0] = -1.
                pressure[1], pressure[-1] = .2, .8
                contact[1] = contact[-1] = 1.
                data = {"tactile": frame("tactile", WujiTactileFrame(24, columns, tuple(pressure))),
                        "contact": frame("contact", WujiTactileFrame(24, columns, tuple(contact)))}
                view.update(self.source(data), now_ns=100)
                matrix = view.heat.get_array()
                self.assertEqual(matrix.shape, (24, columns))
                self.assertTrue(matrix.mask[0, 0])
                self.assertEqual(matrix[0, 1], .2)
                self.assertEqual(matrix[-1, -1], .8)
                self.assertEqual(matrix[0, 2], 0.)  # baseline pressure is not a contact
                self.assertEqual(len(view.contact_outline.get_segments()), 2)
                self.assertIn("CONTACT  ·  2", view.contact_label.get_text())
                self.assertIn("0.800", view.pressure_label.get_text())
                color = view.heat.cmap(view.heat.norm(.2))
                pressure[-1] = .95
                data["tactile"] = frame("tactile", WujiTactileFrame(24, columns, tuple(pressure)))
                view.update(self.source(data), now_ns=100)
                self.assertEqual(view.heat.get_clim(), (0., 1.))
                self.assertEqual(view.heat.cmap(view.heat.norm(.2)), color)
                contact[1] = contact[-1] = 0.
                data["contact"] = frame("contact", WujiTactileFrame(24, columns, tuple(contact)))
                view.update(self.source(data), now_ns=100)
                self.assertIn("NO CONTACT", view.contact_label.get_text())
                self.assertEqual(len(view.contact_outline.get_segments()), 0)
                self.assertEqual(view.heat.get_array().max(), 0.)

    def test_missing_model_or_contact_frame_never_reports_no_contact(self):
        view = self.view()
        data = {"tactile": frame("tactile", WujiTactileFrame(24, 31, (.4,) * 744)),
                "contact": frame("contact", WujiTactileFrame(24, 31, (0.,) * 744))}
        view.update(self.source(data, model=False), now_ns=100)
        self.assertIn("CONTACT UNKNOWN", view.contact_label.get_text())
        self.assertIn("model missing", view.labels[1].get_text())
        self.assertEqual(view.heat.get_array()[0, 0], .4)
        del data["contact"]
        view.update(self.source(data), now_ns=100)
        self.assertIn("CONTACT UNKNOWN", view.contact_label.get_text())
        self.assertIn("contact: waiting", view.labels[1].get_text())

    def test_contact_layout_and_timestamp_must_match_pressure(self):
        view = self.view()
        for contact in (frame("contact", WujiTactileFrame(24, 32, (1.,) * 768)),
                        frame("contact", WujiTactileFrame(24, 31, (1.,) * 744), now_ns=100)):
            data = {"tactile": frame("tactile", WujiTactileFrame(24, 31, (.5,) * 744),
                                     now_ns=200_000_100), "contact": contact}
            view.update(self.source(data), now_ns=200_000_100)
            self.assertIn("CONTACT UNKNOWN", view.contact_label.get_text())
            self.assertEqual(len(view.contact_outline.get_segments()), 0)

    def test_invalid_stale_or_changed_user_clears_contact_and_skeleton(self):
        view = self.view()
        data = {"skeleton": frame("skeleton", skeleton("left")),
                "tactile": frame("tactile", WujiTactileFrame(24, 31, (.5,) * 744)),
                "contact": frame("contact", WujiTactileFrame(24, 31, (1.,) * 744))}
        source = self.source(data)
        view.update(source, now_ns=100)
        self.assertEqual(len(view.contact_outline.get_segments()), 744)
        view.update(source, now_ns=STALE_NS + 101)
        self.assertIn("TACTILE UNAVAILABLE", view.contact_label.get_text())
        self.assertEqual(view.heat.get_array().count(), 0)
        self.assertEqual(len(view.contact_outline.get_segments()), 0)
        self.assertTrue(all(len(line.get_data_3d()[0]) == 0 for line in view.skeleton_lines))
        session = NS(health=lambda: NS(ready=False, detail="SDK user changed"))
        view.update(source, now_ns=100, session=session)
        self.assertIn("SDK user changed", view.title.get_text())
        self.assertEqual(view.heat.get_array().count(), 0)
        source.fault = "invalid skeleton"
        view.update(source, now_ns=100)
        self.assertIn("INVALID", view.labels[0].get_text())
