"""Hand-only observation wiring, including spawn, without device SDKs."""

import multiprocessing as mp
import os
import time
import unittest
from unittest.mock import patch

from bimanual_teleop.control.hand.follow import WujiTeleop, create_wuji_teleop
from bimanual_teleop.control.hand.process import WujiProcess, _create_runtime
from bimanual_teleop.system import SystemState
from bimanual_teleop.types import ControlProfile, JointState
from tests.support.clock import Clock
from tests.support.wuji import Device, Mapper, Sink


class _QueueSink:
    def __init__(self, queue):
        self.queue = queue

    def try_publish(self, sample):
        self.queue.put_nowait((os.getpid(), sample))
        return True

    def try_event(self, event):
        self.queue.put_nowait((os.getpid(), event))
        return True


class _StreamDevice(Device):
    def start(self, sink=None):
        super().start(sink)
        if not self.glove:
            for _ in range(3):
                self.emit()

    def get_latest_stream(self, stream):
        return self.latest


def _recording_runtime(config, *, verbose=False, record_sink=None):
    sides = ("left", "right")
    gloves = {side: _StreamDevice(side, time.monotonic_ns, glove=True) for side in sides}
    hands = {side: _StreamDevice(side, time.monotonic_ns) for side in sides}
    return WujiTeleop(gloves, hands, {side: Mapper() for side in sides},
        profile=ControlProfile("test", "mit", {}), hand_sink=record_sink,
        glove_timeout_s=5., hand_timeout_s=5., threaded=False)


class HandRecordingTests(unittest.TestCase):
    def runtime(self, *, sink=None, hand_sink=None):
        clock = Clock()
        sides = ("left", "right")
        gloves = {side: Device(side, clock, glove=True) for side in sides}
        hands = {side: Device(side, clock) for side in sides}
        runtime = WujiTeleop(gloves, hands, {side: Mapper() for side in sides},
            profile=ControlProfile("test", "mit", {}), sink=sink, hand_sink=hand_sink,
            clock_ns=clock, threaded=False)
        self.addCleanup(runtime.close)
        runtime.start()
        return runtime

    def test_hand_recorder_excludes_glove_and_runtime_events(self):
        recorder = Sink()
        runtime = self.runtime(hand_sink=recorder)
        runtime.engage()
        runtime._step()
        self.assertTrue(all(device.sink is None for device in runtime.gloves.values()))
        self.assertTrue(all(device.sink is recorder for device in runtime.hands.values()))
        self.assertEqual(len(recorder.samples), 2)
        self.assertTrue(all(isinstance(sample.payload, JointState) for sample in recorder.samples))
        self.assertEqual(recorder.events, [])
        self.assertTrue(all(mapper.calls for mapper in runtime.retargeters.values()))
        self.assertTrue(all(hand.commands for hand in runtime.hands.values()))

    def test_omitted_hand_sink_keeps_existing_observer_behavior(self):
        sink = Sink()
        runtime = self.runtime(sink=sink)
        self.assertTrue(all(device.sink is sink for device in
                            (*runtime.gloves.values(), *runtime.hands.values())))
        self.assertEqual(len(sink.samples), 4)
        self.assertTrue(any(event.kind == "wuji.started" for event in sink.events))

    def test_hand_sink_overrides_only_hand_observer(self):
        sink, recorder = Sink(), Sink()
        runtime = self.runtime(sink=sink, hand_sink=recorder)
        self.assertTrue(all(device.sink is sink for device in runtime.gloves.values()))
        self.assertTrue(all(device.sink is recorder for device in runtime.hands.values()))
        self.assertEqual(len(sink.samples), 2)
        self.assertEqual(len(recorder.samples), 2)
        self.assertTrue(sink.events)
        self.assertEqual(recorder.events, [])

    def test_recording_factory_does_not_enable_emf_or_connect(self):
        config = {"parameters": {"kp": 5., "kd": .05, "current_limit_a": 1.5},
                  "devices": {side: {"glove": "fake-glove", "hand": "fake-hand"}
                              for side in ("left", "right")}}
        recorder = Sink()
        with patch("bimanual_teleop.devices.wuji.adapter._sdk_module",
                   side_effect=AssertionError("SDK must not load during construction")):
            runtime = create_wuji_teleop(config, hand_sink=recorder)
        self.assertIs(runtime.hand_sink, recorder)
        self.assertIsNone(runtime.sink)
        self.assertIsNone(runtime.session.manager)
        self.assertTrue(all(glove.streams == ("skeleton",) for glove in runtime.gloves.values()))

    def test_child_factory_routes_recording_to_hand_sink(self):
        recorder = Sink()
        with patch("bimanual_teleop.common.console.configure_runtime_logging"), \
                patch("bimanual_teleop.control.hand.follow.create_wuji_teleop") as create:
            _create_runtime({}, record_sink=recorder)
            create.assert_called_once_with({}, hand_sink=recorder)
        with patch("bimanual_teleop.common.console.configure_runtime_logging"), \
                patch("bimanual_teleop.control.hand.follow.create_wuji_teleop") as create:
            _create_runtime({})
            create.assert_called_once_with({})

    def test_spawn_publishes_original_hand_samples_without_snapshot_polling(self):
        context = mp.get_context("spawn")
        queue = context.Queue(maxsize=32)
        runtime = WujiProcess({"hand_timeout_s": 5., "glove_timeout_s": 5.},
                              record_sink=_QueueSink(queue), _runtime_factory=_recording_runtime)
        try:
            runtime.start()
            # All four source samples per hand arrive before the first snapshot;
            # collecting them does not require status() or glove_samples().
            samples = [queue.get(timeout=5.) for _ in range(8)]
            self.assertTrue(all(pid == runtime._process.pid and pid != os.getpid()
                                for pid, _ in samples))
            for side in ("left", "right"):
                selected = [sample for _, sample in samples
                            if sample.header.ref.stream == f"wuji_{side}_hand"]
                self.assertEqual([sample.header.ref.sequence for sample in selected], [1, 2, 3, 4])
                self.assertTrue(all(sample.header.received_monotonic_ns > 0 for sample in selected))
                self.assertTrue(all(isinstance(sample.payload, JointState) for sample in selected))
            runtime.prepare_engage()
            runtime.begin_follow()
            self.assertEqual(runtime.state, SystemState.ENGAGED)
            runtime.pause("recording stopped")
            runtime.prepare_engage()
            runtime.begin_follow()
            self.assertEqual(runtime.state, SystemState.ENGAGED)
            runtime.close()
            queue.put_nowait("parent still owns recorder")
            self.assertEqual(queue.get(timeout=5.), "parent still owns recorder")
        finally:
            runtime.close()
            queue.close()
            queue.join_thread()


if __name__ == "__main__":
    unittest.main()
