"""Recorder acknowledgements, shutdown and UI ordering without device connections."""

import json
import multiprocessing as mp
from pathlib import Path
from queue import Empty, Queue
import tempfile
import threading
import time
from types import SimpleNamespace
import unittest
from unittest.mock import Mock, patch

import numpy as np
import zarr

from bimanual_teleop.recording.config import RecordingConfig
from bimanual_teleop.recording.convert import _read_stream
from bimanual_teleop.recording.recorder import Recorder, _worker
from bimanual_teleop.recording.sink import COMMAND_STREAMS, STATE_STREAMS, Record
from bimanual_teleop.recording.storage import EpisodeWriter
from bimanual_teleop.recording.ui import RecordingUI
from bimanual_teleop.system import SystemState
from tests.test_recording_sink import _channel
from tests.test_recording_storage import _Kinematics


class _Rig:
    def __init__(self):
        self.metadata, self.closed = {}, threading.Event()
        self.frames = Queue()

    def start(self):
        pass

    def poll(self):
        frames = []
        while True:
            try:
                frames.append(self.frames.get_nowait())
            except Empty:
                return frames

    def frame_set(self, stamp):
        for index in range(3):
            camera = f"camera_{index}"
            record = Record(f"cameras/{camera}/rgb", stamp, stamp, {"source_time_ms": stamp / 1e6})
            self.frames.put((camera, "rgb", np.zeros((480, 640, 3), dtype="u1"), record))

    def close(self):
        self.closed.set()


class RecordingLifecycleTests(unittest.TestCase):
    def setUp(self):
        temporary = tempfile.TemporaryDirectory()
        self.addCleanup(temporary.cleanup)
        self.path = Path(temporary.name) / "episode"
        self.config = RecordingConfig(("main", "left", "right"), main_depth=False)

    def worker(self, *, delay=0., failure=False):
        channel, rig, parent_alive = _channel(512), _Rig(), threading.Event()
        parent_alive.set()
        channel.active.value = False
        connection, child = mp.Pipe()
        seen = Queue()
        def factory(*args):
            writer = EpisodeWriter(*args)
            append = writer.append
            def observed(record):
                if failure:
                    raise OSError("simulated disk failure")
                if delay:
                    time.sleep(delay)
                append(record)
                seen.put(record)
            writer.append = observed
            return writer
        kine = _Kinematics()
        kine.model = SimpleNamespace(digest="test-model")
        for patcher in (patch("bimanual_teleop.recording.camera.CameraRig", return_value=rig),
                        patch("bimanual_teleop.devices.tianji.model.TianjiKinematics", return_value=kine),
                        patch("bimanual_teleop.recording.storage.EpisodeWriter", side_effect=factory),
                        patch("bimanual_teleop.recording.recorder.mp.parent_process",
                              return_value=SimpleNamespace(is_alive=parent_alive.is_set))):
            patcher.start()
            self.addCleanup(patcher.stop)
        thread = threading.Thread(target=_worker,
            args=(self.config, None, {}, channel, child, False), daemon=True)
        thread.start()
        def cleanup():
            if thread.is_alive():
                parent_alive.clear()
                thread.join(2.)
            connection.close()
            self.assertFalse(thread.is_alive(), "worker cleanup exceeded two seconds")
        self.addCleanup(cleanup)
        self.assertEqual(self.receive(connection), ("ready", None))
        connection.send(("start", 7, str(self.path), 1000))
        self.assertEqual(self.receive(connection), ("recording", str(self.path)))
        return channel, connection, thread, parent_alive, rig, seen

    def receive(self, connection):
        self.assertTrue(connection.poll(3.), "recorder acknowledgement timed out")
        return connection.recv()

    def record(self, stamp, sequence=1, stream="hands/left"):
        arm = stream.startswith(("arms/", "arm_commands/"))
        values = {"joint_pos": (.1,) * (7 if arm else 20)}
        if stream.startswith("arms/"):
            values["wrench"] = (0.,) * 6
        if stream.startswith("arm_commands/"):
            values["eef_pose"] = (0.,) * 6 + (1.,)
        return Record(stream, stamp, sequence, values)

    def enqueue(self, channel, record, generation=7):
        channel.sent[(STATE_STREAMS + COMMAND_STREAMS).index(record.stream)] += 1
        channel.queue.put((generation, record))

    def seed(self, channel, rig, exclude=()):
        for stream in STATE_STREAMS + COMMAND_STREAMS:
            if stream not in exclude:
                self.enqueue(channel, self.record(1001, 0, stream))
        rig.frame_set(1002)

    def test_worker_save_ack_follows_flush_and_effective_end_excludes_late_data(self):
        channel, connection, thread, _, rig, seen = self.worker()
        self.seed(channel, rig)
        self.enqueue(channel, self.record(1001), generation=6)  # Ignore earlier episode.
        self.enqueue(channel, self.record(1020))  # Already written before stop arrives.
        while seen.get(timeout=2.).time_ns != 1020:
            pass
        connection.send(("stop", 1010, "complete", None))
        self.enqueue(channel, self.record(1030, 2))
        self.enqueue(channel, self.record(1005, 1, "hands/right"))
        rig.frame_set(3000)
        self.assertEqual(self.receive(connection), ("saved", (str(self.path), "complete")))
        document = json.loads((self.path / "episode.json").read_text())
        self.assertEqual((document["status"], document["end_ns"]), ("complete", 1010))
        raw = zarr.open_group(str(self.path / "raw.zarr"), mode="r")
        self.assertEqual(raw["hands/right/time_ns"][:].tolist(), [1001, 1005])
        # Raw may retain a previously written tail; conversion must honor the manifest window.
        selected = _read_stream(raw, "hands/left", {"joint_pos": 20}, 1000, 1010)
        self.assertEqual(selected.times.tolist(), [1001])
        self.assertNotIn(1030, raw["hands/left/time_ns"][:])
        connection.send(("close",))
        thread.join(2.)
        self.assertFalse(thread.is_alive())
        self.assertTrue(rig.closed.is_set())

    def test_slow_writer_cannot_ack_complete_while_valid_tail_records_are_lost(self):
        channel, connection, _, _, rig, _ = self.worker(delay=.002)
        self.seed(channel, rig, exclude=("hands/left",))
        connection.send(("stop", 2000, "complete", None))
        for index in range(260):
            self.enqueue(channel, self.record(1001 + index, index))
        rig.frame_set(3000)
        self.assertEqual(self.receive(connection), ("saved", (str(self.path), "complete")))
        document = json.loads((self.path / "episode.json").read_text())
        self.assertEqual(document["status"], "complete")
        raw = zarr.open_group(str(self.path / "raw.zarr"), mode="r")
        self.assertEqual(raw["hands/left/sequence"][:].tolist(), list(range(260)))

    def test_parent_death_and_writer_failure_close_resources_without_complete_episode(self):
        for failure in (False, True):
            with self.subTest(writer_failure=failure):
                self.path = self.path.parent / f"failure_{failure}"
                channel, _, thread, parent_alive, rig, _ = self.worker(failure=failure)
                if failure:
                    self.enqueue(channel, self.record(1001))
                else:
                    parent_alive.clear()
                thread.join(2.)
                self.assertFalse(thread.is_alive())
                self.assertTrue(channel.failed.is_set())
                self.assertFalse(channel.active.value)
                self.assertTrue(rig.closed.is_set())
                self.assertEqual(json.loads((self.path / "episode.json").read_text())["status"], "failed")

    def test_missing_camera_tail_is_bounded_and_marks_episode_failed(self):
        channel, connection, _, _, rig, _ = self.worker()
        self.seed(channel, rig)
        connection.send(("stop", 2000, "complete", None))
        self.assertEqual(self.receive(connection), ("saved", (str(self.path), "failed")))
        document = json.loads((self.path / "episode.json").read_text())
        self.assertEqual(document["status"], "failed")
        self.assertTrue(channel.failed.is_set())
        self.assertFalse(channel.active.value)

    def coordinator(self):
        recorder = Recorder.__new__(Recorder)
        recorder.channel, recorder.connection, recorder.process = _channel(), Mock(), Mock()
        recorder.process.is_alive.return_value = True
        recorder.state, recorder.error, recorder.ready = "recording", None, True
        recorder.notices, recorder.session = [], self.path
        return recorder

    def test_late_recording_ack_does_not_cancel_saving_and_end_waits_for_saved_ack(self):
        recorder = self.coordinator()
        parent, child = mp.Pipe()
        self.addCleanup(parent.close)
        self.addCleanup(child.close)
        recorder.connection, recorder.state = parent, "idle"
        recorder.channel.latest_ns[:] = [time.monotonic_ns()] * 8
        recorder.begin()
        self.assertEqual(recorder.state, "starting")
        self.assertEqual(self.receive(child)[0], "start")
        recorder.end()
        self.assertEqual(self.receive(child)[0], "stop")
        child.send(("recording", str(self.path)))
        recorder.poll()
        self.assertEqual(recorder.state, "saving")
        child.send(("saved", (str(self.path), "complete")))
        recorder.poll()
        self.assertEqual(recorder.state, "idle")

    def test_manual_pause_saves_fault_pause_fails_and_recovery_pauses_before_join(self):
        for mode in ("keyboard", "gesture", "fault", "recover", "broken_pipe"):
            with self.subTest(mode=mode):
                recorder, runtime = self.coordinator(), SimpleNamespace(state=SystemState.ENGAGED, last_error=None)
                runtime.pause = lambda reason: setattr(runtime, "state", SystemState.PAUSED)
                ui = RecordingUI(runtime, None, recorder=recorder, emit=lambda _: None)
                recorder.recover = Mock(side_effect=lambda: self.assertEqual(runtime.state, SystemState.PAUSED))
                if mode == "keyboard":
                    ui.handle(" ")
                elif mode == "gesture":
                    ui.handle_gesture("pause")
                elif mode == "fault":
                    runtime.state, runtime.last_error = SystemState.PAUSED, "force feedback lost"
                    ui.report_runtime_pause()
                elif mode == "broken_pipe":
                    recorder.connection.send.side_effect = BrokenPipeError("writer exited")
                    ui.abort("writer exited")
                    self.assertEqual(runtime.state, SystemState.PAUSED)
                    self.assertTrue(recorder.channel.failed.is_set())
                else:
                    recorder.error = "writer unavailable"
                    ui.handle("c")
                    recorder.recover.assert_called_once()
                sent, = recorder.connection.send.call_args_list
                self.assertEqual(sent.args[0][2], "complete" if mode in ("keyboard", "gesture") else "failed")

    def test_stuck_process_shutdown_has_bounded_join_escalation(self):
        recorder = self.coordinator()
        process = recorder.process
        recorder._stop_process()
        self.assertEqual([args.args[0] for args in process.join.call_args_list], [3., 2., 1.])
        process.terminate.assert_called_once()
        process.kill.assert_called_once()
        self.assertIsNone(recorder.process)
        self.assertIsNone(recorder.connection)
