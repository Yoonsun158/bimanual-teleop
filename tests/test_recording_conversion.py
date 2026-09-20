"""Offline conversion tests with actual encoded video and asynchronous streams."""

import importlib.util
import json
from pathlib import Path
import tempfile
import unittest

import numpy as np
from scipy.spatial.transform import Rotation

from bimanual_teleop.recording.convert import convert_recordings


START_NS = 1_000_000_000


def write_video(path, count, camera=0):
    import av

    with av.open(str(path), "w") as container:
        stream = container.add_stream("libx264rgb", rate=30)
        stream.width = stream.height = 16
        stream.pix_fmt = "rgb24"
        stream.options = {"crf": "0", "preset": "ultrafast"}
        for index in range(count):
            pixels = np.zeros((16, 16, 3), np.uint8)
            pixels[..., 0] = 20 + index * 20
            pixels[..., 1] = camera * 50
            frame = av.VideoFrame.from_ndarray(pixels, format="rgb24")
            for packet in stream.encode(frame):
                container.mux(packet)
        for packet in stream.encode():
            container.mux(packet)


def make_episode(parent, name="episode_000000", *, main=(10., 43., 77., 110.),
                 state=None, commands=None, other=None, depth=False):
    import zarr

    path = parent / name
    path.mkdir(parents=True)
    stop = max(main) + 41
    state = np.arange(0, stop, 10.) if state is None else np.asarray(state)
    commands = np.arange(0, stop, 20.) if commands is None else np.asarray(commands)
    metadata = {"model": "test-model", "joint_unit": "rad"}
    descriptor = {"schema_version": 1, "status": "complete", "start_ns": START_NS,
                  "end_ns": START_NS + round(stop * 1e6), "metadata": metadata}
    (path / "episode.json").write_text(json.dumps(descriptor))
    raw = zarr.open_group(str(path / "raw.zarr"), mode="w")
    raw.attrs["metadata"] = metadata

    def group(name, times):
        result = raw.create_group(name)
        times = np.asarray(times, dtype=float)
        result.array("time_ns", START_NS + np.rint(times * 1e6).astype(np.int64))
        result.array("sequence", np.arange(len(times), dtype=np.int64))
        return result

    for side, base in (("left", 0.), ("right", 100.)):
        arm = group(f"arms/{side}", state)
        arm.array("joint_pos", base + np.arange(7) + state[:, None] / 1000.)
        poses = np.zeros((len(state), 7))
        poses[:, 0] = base + state / 1000.
        poses[:, 3:] = Rotation.from_euler("z", state / 1000.).as_quat()
        arm.array("eef_pose", poses)
        arm.array("wrench", base + np.arange(6) + state[:, None] / 1000.)
        hand = group(f"hands/{side}", state)
        hand.array("joint_pos", base + np.arange(20) + state[:, None] / 1000.)
        command = group(f"arm_commands/{side}", commands)
        command.array("joint_pos", base + 10 + np.arange(7) + commands[:, None] / 1000.)
        goals = np.zeros((len(commands), 7))
        goals[:, 0] = base + 5 + commands / 1000.
        goals[:, 6] = 1.
        command.array("eef_pose", goals)
        hand_command = group(f"hand_commands/{side}", commands)
        hand_command.array("joint_pos", base + 20 + np.arange(20) + commands[:, None] / 1000.)
    for camera in range(3):
        times = main if not other or camera not in other else other[camera]
        rgb = group(f"cameras/camera_{camera}/rgb", times)
        rgb.array("source_time_ms", np.asarray(times, float))
        write_video(path / f"camera_{camera}.mp4", len(times), camera)
    if depth:
        times = np.asarray(main) + 2
        frames = group("cameras/camera_0/depth", times)
        frames.array("source_time_ms", times)
        frames.array("image", np.stack([np.full((4, 4), i + 100, np.uint16) for i in range(len(times))]))
    return path, raw


@unittest.skipUnless(importlib.util.find_spec("av") and importlib.util.find_spec("zarr"),
                     "recording dependencies are not installed")
class RecordingConversionTests(unittest.TestCase):
    def setUp(self):
        self.temporary = tempfile.TemporaryDirectory()
        self.addCleanup(self.temporary.cleanup)
        self.root = Path(self.temporary.name)
        self.input = self.root / "raw"
        self.output = self.root / "dataset.zarr"

    def convert(self, action_space="eef"):
        import zarr

        report = convert_recordings(self.input, self.output, action_space=action_space)
        return zarr.open_group(str(self.output), mode="r"), report

    def test_eef_actions_are_controller_goals_and_observations_are_interpolated(self):
        make_episode(self.input, depth=True)
        dataset, report = self.convert()
        data = dataset["data"]
        np.testing.assert_allclose(data["timestamp"][:], [.010, .043, .077, .110])
        np.testing.assert_allclose(data["robot_joint"][:, 0], [.010, .043, .077, .110])
        np.testing.assert_allclose(data["robot_joint"][:, 7], [100.010, 100.043, 100.077, 100.110], atol=1e-5)
        np.testing.assert_allclose(data["robot_eef_pose"][:, 5], [.010, .043, .077, .110], atol=1e-6)
        self.assertEqual(data["action"].shape, (4, 52))
        np.testing.assert_allclose(data["action"][:, 0], [5., 5.040, 5.060, 5.100], atol=1e-6)
        np.testing.assert_allclose(data["action"][:, 6], [105., 105.040, 105.060, 105.100], atol=1e-5)
        np.testing.assert_allclose(data["action"][0, 12:32], 20 + np.arange(20))
        np.testing.assert_allclose(data["action"][0, 32:], 120 + np.arange(20))
        np.testing.assert_array_equal(data["camera_2"][1, 0, 0], [40, 100, 0])
        np.testing.assert_array_equal(data["camera_0_depth"][:, 0, 0], [100, 101, 102, 103])
        self.assertEqual(data["camera_0"].dtype, np.dtype("uint8"))
        self.assertEqual(data["robot_joint"].dtype, np.dtype("float32"))
        self.assertEqual(data["timestamp"].dtype, np.dtype("float64"))
        np.testing.assert_array_equal(dataset["meta/episode_ends"][:], [4])
        self.assertEqual(report["output_frames"], 4)
        self.assertEqual(dataset["meta"].attrs["quality_report"]["episodes"][0]["metadata"]["model"], "test-model")

    def test_joint_action_layout_and_failed_episodes_are_skipped(self):
        make_episode(self.input)
        failed = self.input / "episode_000001"
        failed.mkdir()
        (failed / "episode.json").write_text(json.dumps({"status": "failed"}))
        dataset, report = self.convert("joint")
        actions = dataset["data/action"]
        self.assertEqual(actions.shape, (4, 54))
        np.testing.assert_array_equal(actions[0, :7], 10 + np.arange(7))
        np.testing.assert_array_equal(actions[0, 7:14], 110 + np.arange(7))
        np.testing.assert_array_equal(actions[0, 14:34], 20 + np.arange(20))
        self.assertEqual(report["episodes"][1]["status"], "failed")

    def test_missing_camera_and_invalid_force_split_without_joining_gaps(self):
        _, raw = make_episode(self.input, main=(10, 40, 70, 100, 130, 160),
                              other={1: (10, 40, 100, 130, 160)})
        raw["arms/left/wrench"][13, 0] = np.nan  # Exactly at 130 ms.
        dataset, report = self.convert()
        np.testing.assert_allclose(dataset["data/timestamp"][:], [.01, .04, .10, .16])
        np.testing.assert_array_equal(dataset["meta/episode_ends"][:], [2, 3, 4])
        self.assertEqual(report["episodes"][0]["invalid_reasons"]["camera_1_unmatched"], 1)
        self.assertEqual(dataset["meta"].attrs["segments"][1]["reference_frame_start"], 3)

    def test_large_main_camera_gap_starts_a_new_episode(self):
        make_episode(self.input, main=(10, 40, 120, 150))
        dataset, report = self.convert()
        np.testing.assert_array_equal(dataset["meta/episode_ends"][:], [2, 4])
        self.assertEqual(report["episodes"][0]["main_camera_gaps"], 1)

    def test_state_interpolation_never_bridges_more_than_50ms_or_extrapolates(self):
        make_episode(self.input, main=(0, 30, 60, 90), state=(0, 60), commands=(0, 30, 60, 90))
        dataset, _ = self.convert()
        np.testing.assert_allclose(dataset["data/timestamp"][:], [0., .06])
        np.testing.assert_array_equal(dataset["meta/episode_ends"][:], [1, 2])

    def test_commands_do_not_use_future_samples_or_hold_stale_values(self):
        make_episode(self.input, main=(10, 40, 80, 100), commands=(20, 100))
        dataset, _ = self.convert()
        np.testing.assert_allclose(dataset["data/timestamp"][:], [.04, .10])
        np.testing.assert_allclose(dataset["data/action"][:, 0], [5.02, 5.10], atol=1e-6)
        np.testing.assert_array_equal(dataset["meta/episode_ends"][:], [1, 2])

    def test_pose_interpolation_uses_shortest_rotation_path(self):
        _, raw = make_episode(self.input, main=(20,), state=(0, 40), commands=(0, 40))
        poses = raw["arms/left/eef_pose"][:]
        poses[:, 3:] = Rotation.from_euler("z", [170, -170], degrees=True).as_quat()
        raw["arms/left/eef_pose"][:] = poses
        dataset, _ = self.convert()
        self.assertAlmostEqual(abs(float(dataset["data/robot_eef_pose"][0, 5])), np.pi, places=6)

    def test_invalid_quaternion_is_excluded_without_repairing_it(self):
        _, raw = make_episode(self.input, main=(10, 40, 70))
        raw["arms/right/eef_pose"][4, 3:] = 0.
        dataset, report = self.convert()
        np.testing.assert_allclose(dataset["data/timestamp"][:], [.01, .07])
        self.assertEqual(report["episodes"][0]["invalid_reasons"]["right_robot_eef_pose_invalid_or_gap"], 1)

    def test_episode_bounds_are_half_open_and_reference_time_is_relative(self):
        path, _ = make_episode(self.input, main=(0, 10, 40, 70))
        manifest = path / "episode.json"
        descriptor = json.loads(manifest.read_text())
        descriptor.update(start_ns=START_NS + 10_000_000, end_ns=START_NS + 70_000_000)
        manifest.write_text(json.dumps(descriptor))
        # A command at 10 ms is necessary; a command before the episode is not reused.
        import zarr
        raw = zarr.open_group(str(path / "raw.zarr"), mode="a")
        for kind in ("arm_commands", "hand_commands"):
            for side in ("left", "right"):
                raw[f"{kind}/{side}/time_ns"][0] = START_NS + 10_000_000
        dataset, _ = self.convert()
        np.testing.assert_allclose(dataset["data/timestamp"][:], [0., .03])

    def test_frame_count_mismatch_fails_without_publishing_partial_output(self):
        path, _ = make_episode(self.input)
        write_video(path / "camera_2.mp4", 3, 2)
        with self.assertRaisesRegex(ValueError, "decoded 3 frames"):
            self.convert()
        self.assertFalse(self.output.exists())
        self.assertFalse(list(self.root.glob(".*.converting-*")))

    def test_existing_output_is_never_overwritten(self):
        make_episode(self.input)
        self.output.mkdir()
        marker = self.output / "important.txt"
        marker.write_text("keep me")
        with self.assertRaises(FileExistsError):
            self.convert()
        self.assertEqual(marker.read_text(), "keep me")

    def test_nonmonotonic_timestamps_are_rejected(self):
        _, raw = make_episode(self.input)
        raw["hands/right/time_ns"][1] = raw["hands/right/time_ns"][0]
        with self.assertRaisesRegex(ValueError, "strictly increasing"):
            self.convert()
        self.assertFalse(self.output.exists())

    def test_cli_requires_action_space(self):
        from bimanual_teleop.cli.convert_recording import main
        from contextlib import redirect_stderr
        import io

        with redirect_stderr(io.StringIO()), self.assertRaises(SystemExit) as caught:
            main(["--input", str(self.input), "--output", str(self.output)])
        self.assertEqual(caught.exception.code, 2)


if __name__ == "__main__":
    unittest.main()
