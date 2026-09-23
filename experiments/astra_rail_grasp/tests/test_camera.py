"""No device access: synthetic frames, SDK-shaped fakes, and fake subprocess."""

import json
from pathlib import Path
import tempfile
import time
from types import SimpleNamespace as NS
import unittest
from unittest.mock import Mock, patch

import numpy as np
from PIL import Image

from experiments.astra_rail_grasp.camera import (
    CameraCapture, _capture_worker, _depth_meters, _extract_frame, _open_camera, _simulated_frame,
)


def inject(capture, *, age=0., problem=None, serial="SIMULATED-0"):
    frame = _simulated_frame(12)
    frame["captured_monotonic_ns"] -= int(age * 1e9)
    capture._accept_packet({"cameras": {serial: {
        "serial": serial, "name": "SIMULATED TEST CAMERA", "simulated": True,
        "problem": problem, "depth_enabled": True, "frame": frame}}, "problems": []})
    return frame


class CameraTests(unittest.TestCase):
    def test_invalid_depth_is_nan_and_scale_is_meters(self):
        depth, fraction = _depth_meters(np.array([[0, 250, 1000], [-1, np.inf, np.nan]]), .001)
        self.assertAlmostEqual(float(depth[0, 1]), .25)
        self.assertAlmostEqual(float(depth[0, 2]), 1.)
        self.assertTrue(np.isnan(depth[0, 0]))
        self.assertTrue(np.isnan(depth[1]).all())
        self.assertAlmostEqual(fraction, 2 / 6)
        depth, fraction = _depth_meters(np.ones((2, 2)), -1.)
        self.assertTrue(np.isnan(depth).all())
        self.assertEqual(fraction, 0.)

    def test_health_does_not_write_and_stale_is_not_ready(self):
        with tempfile.TemporaryDirectory() as directory:
            destination = Path(directory) / "does-not-exist"
            capture = CameraCapture(destination, fake=True)
            self.assertFalse(capture.health()["ready"])
            inject(capture)
            self.assertTrue(capture.health()["ready"])
            inject(capture, age=.6)
            health = capture.health()
            self.assertFalse(health["ready"])
            self.assertIn("stale", " ".join(health["problems"]))
            self.assertFalse(destination.exists())

    def test_snapshot_preserves_unknown_depth_and_explicit_simulation(self):
        with tempfile.TemporaryDirectory() as directory:
            capture = CameraCapture(Path(directory), fake=True)
            original = inject(capture)
            snap = capture.snapshot()
            self.assertTrue(snap["simulated"])
            self.assertTrue(snap["ready"])
            row = snap["cameras"][0]
            self.assertIn("SIMULATED", row["name"])
            self.assertEqual(row["sequence"], 12)
            self.assertGreaterEqual(row["age_s"], 0.)
            for key in ("rgb_path", "depth_path"):
                self.assertTrue(Path(row[key]).is_absolute())
                self.assertTrue(Path(row[key]).exists())
            np.testing.assert_array_equal(np.asarray(Image.open(row["rgb_path"])), original["rgb"])
            depth = np.load(row["depth_path"], allow_pickle=False)
            self.assertTrue(np.isnan(depth[:40]).all())
            self.assertTrue((depth[40:] == 1.).all())
            saved = json.loads(Path(snap["json_path"]).read_text())
            self.assertEqual(saved, snap)

    def test_one_fresh_camera_suffices_and_failures_remain_visible(self):
        with tempfile.TemporaryDirectory() as directory:
            capture = CameraCapture(Path(directory), fake=True)
            inject(capture)
            capture._cameras["busy"] = {"serial": "busy", "name": "Unavailable",
                "simulated": False, "problem": "device busy", "frame": None}
            result = capture.health()
            self.assertTrue(result["ready"])
            self.assertIn("device busy", " ".join(result["problems"]))
            capture.close()
            self.assertFalse(capture.health()["ready"])

    def test_stream_failure_immediately_invalidates_even_a_recent_frame(self):
        with tempfile.TemporaryDirectory() as directory:
            capture = CameraCapture(Path(directory), fake=True)
            inject(capture)
            capture._cameras["SIMULATED-0"].update(stream_failed=True, problem="disconnected")
            result = capture.health()
            self.assertFalse(result["ready"])
            self.assertIn("disconnected", " ".join(result["problems"]))

    def test_health_only_reads_parent_cache(self):
        with tempfile.TemporaryDirectory() as directory:
            capture = CameraCapture(Path(directory), fake=True)
            capture._queue = Mock()
            inject(capture)
            self.assertTrue(capture.health()["ready"])
            capture._queue.get.assert_not_called()
            capture._queue.get_nowait.assert_not_called()

    def test_rgb_only_fallback_stays_on_requested_serial(self):
        first, second = Mock(), Mock()
        first.start.side_effect = RuntimeError("no depth stream")
        rs = NS(pipeline=Mock(side_effect=[first, second]), config=Mock(side_effect=[Mock(), Mock()]),
                stream=NS(color="color", depth="depth"), format=NS(rgb8="rgb8", z16="z16"))
        result = _open_camera(rs, "context", "123456")
        self.assertIs(result["pipeline"], second)
        self.assertFalse(result["depth_enabled"])
        self.assertIn("no depth stream", result["depth_problem"])
        first.stop.assert_called_once()
        for camera in (first, second):
            camera.start.call_args.args[0].enable_device.assert_called_once_with("123456")

    def test_busy_device_is_an_observation_not_exception(self):
        pipeline = Mock()
        pipeline.start.side_effect = RuntimeError("device busy")
        rs = NS(pipeline=Mock(return_value=pipeline), config=Mock(), stream=NS(color=1, depth=2),
                format=NS(rgb8=1, z16=2))
        result = _open_camera(rs, None, "busy")
        self.assertIn("device busy", result["problem"])
        self.assertNotIn("pipeline", result)

    def test_no_devices_worker_reports_a_problem_without_opening_pipeline(self):
        context = Mock()
        context.query_devices.return_value = []
        rs = NS(context=Mock(return_value=context), pipeline=Mock())
        stopped = Mock()
        stopped.is_set.side_effect = [False, True]
        out = Mock()
        with patch.dict("sys.modules", {"pyrealsense2": rs}):
            _capture_worker(out, stopped, False)
        rs.pipeline.assert_not_called()
        packet = out.put_nowait.call_args.args[0]
        self.assertEqual(packet["cameras"], {})
        self.assertIn("No RealSense", " ".join(packet["problems"]))

    def test_extract_aligned_depth_has_matching_metadata(self):
        intr = NS(width=2, height=1, fx=500., fy=501., ppx=1., ppy=.5, model="none", coeffs=[0.] * 5)
        profile = Mock()
        profile.as_video_stream_profile.return_value.get_intrinsics.return_value = intr
        color = Mock(profile=profile)
        color.get_data.return_value = np.zeros((1, 2, 3), dtype=np.uint8)
        color.get_frame_number.return_value = 3
        color.get_timestamp.return_value = 1234.5
        color.get_frame_timestamp_domain.return_value = "hardware_clock"
        depth = Mock(profile=profile)
        depth.get_data.return_value = np.array([[0, 600]], dtype=np.uint16)
        depth.get_frame_number.return_value = 4
        depth.get_timestamp.return_value = 1234.6
        frames = Mock()
        frames.get_color_frame.return_value = color
        frames.get_depth_frame.return_value = depth
        align = Mock()
        align.process.return_value = frames
        record, problem = _extract_frame({"align": align, "depth_enabled": True, "depth_scale": .001}, frames, 9)
        self.assertIsNone(problem)
        self.assertEqual(record["frame_number"], 3)
        self.assertEqual(record["depth_frame_number"], 4)
        self.assertEqual(record["depth_valid_fraction"], .5)
        self.assertTrue(np.isnan(record["depth"][0, 0]))
        self.assertAlmostEqual(float(record["depth"][0, 1]), .6)
        self.assertEqual(record["intrinsics"]["fx"], 500.)
        align.process.assert_called_once_with(frames)

    def test_snapshot_disk_failure_is_reported(self):
        with tempfile.TemporaryDirectory() as directory:
            destination = Path(directory) / "file"
            destination.write_text("not a directory")
            capture = CameraCapture(destination, fake=True)
            inject(capture)
            result = capture.snapshot()
            self.assertIsNone(result["json_path"])
            self.assertIn("could not be saved", " ".join(result["problems"]))

    def test_fake_worker_lifecycle_does_not_import_or_open_realsense(self):
        with tempfile.TemporaryDirectory() as directory:
            capture = CameraCapture(Path(directory), fake=True)
            try:
                capture.start()
                deadline = time.monotonic() + 8.
                while time.monotonic() < deadline and not capture.health()["ready"]:
                    time.sleep(.03)
                state = capture.health()
                self.assertTrue(state["ready"], state)
                self.assertTrue(state["simulated"])
                self.assertFalse(any(Path(directory).iterdir()))
                self.assertTrue(capture.snapshot()["ready"])
            finally:
                capture.close()
            self.assertFalse(capture._process.is_alive())
            capture.close()


if __name__ == "__main__":
    unittest.main()
