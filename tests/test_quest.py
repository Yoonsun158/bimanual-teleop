"""Quest protocol and USB lifecycle checks without hardware."""

from __future__ import annotations

import io
import json
from pathlib import Path
import subprocess
import sys
import unittest
from unittest.mock import Mock, patch


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from bimanual_teleop.devices.quest.adapter import (  # noqa: E402
    QuestSource,
    decode_message,
    select_usb_serial,
)
from bimanual_teleop.types import Event  # noqa: E402
from bimanual_teleop.control.arm.quest import QuestInputMonitor  # noqa: E402


from tests.support.quest_protocol import frame, sample, reference_event


class Sink:
    def __init__(self) -> None:
        self.samples = []
        self.events = []

    def try_publish(self, value) -> bool:
        self.samples.append(value)
        return True

    def try_event(self, value) -> bool:
        self.events.append(value)
        return True


class QuestProtocolTests(unittest.TestCase):
    def test_positions_sides_and_single_query_timestamp(self) -> None:
        result = sample()
        self.assertEqual(result.payload.left.position_m, (-3.0, -1.0, 2.0))
        self.assertEqual(result.payload.right.position_m, (-2.0, 1.0, 4.0))
        self.assertEqual(result.payload.head.position_m, (0.0, 0.0, 1.0))
        self.assertEqual(result.header.source_time.value, 1_000_000_000)
        self.assertEqual(result.header.received_monotonic_ns, 3_000_000_000)
        self.assertEqual(result.payload.xr_time, 2_000_000_000)
        self.assertEqual(result.payload.send_monotonic_ns, 1_001_000_000)
        self.assertIn("query", result.header.source_time.meaning)

    def test_orientation_basis_change_on_three_known_axes(self) -> None:
        data = frame()
        # A 120-degree rotation about (1,1,1) maps OpenXR x -> y -> z -> x.
        data["left"]["q"] = [0.5, 0.5, 0.5, 0.5]
        q = decode_message(json.dumps(data), 1).payload.left.orientation_xyzw

        def rotated(vector):
            x, y, z, w = q
            vx, vy, vz = vector
            return (
                (1 - 2*y*y - 2*z*z)*vx + (2*x*y - 2*z*w)*vy + (2*x*z + 2*y*w)*vz,
                (2*x*y + 2*z*w)*vx + (1 - 2*x*x - 2*z*z)*vy + (2*y*z - 2*x*w)*vz,
                (2*x*z - 2*y*w)*vx + (2*y*z + 2*x*w)*vy + (1 - 2*x*x - 2*y*y)*vz,
            )

        # In FLU, those same axes are -Y, +Z, -X, respectively.
        for source_axis, expected in [
            ((0, -1, 0), (0, 0, 1)),
            ((0, 0, 1), (-1, 0, 0)),
            ((-1, 0, 0), (0, -1, 0)),
        ]:
            with self.subTest(axis=source_axis):
                for actual, desired in zip(rotated(source_axis), expected):
                    self.assertAlmostEqual(actual, desired)

    def test_partial_invalid_pose_keeps_available_component_and_other_hand(self) -> None:
        data = frame()
        data["left"].update(p=None, flags=5)
        result = decode_message(json.dumps(data), 1)
        self.assertIsNone(result.payload.left.position_m)
        self.assertIsNotNone(result.payload.left.orientation_xyzw)
        self.assertFalse(result.payload.left.position_valid)
        self.assertTrue(result.payload.left.orientation_tracked)
        self.assertFalse(result.payload.left.valid)
        self.assertTrue(result.payload.right.valid)

    def test_corrupt_lines_are_rejected_without_inventing_pose(self) -> None:
        for line in [json.dumps(frame())[:-12], '{"v": 9, "type": "frame"}', "not-json"]:
            with self.subTest(line=line[:30]), self.assertRaises(ValueError):
                decode_message(line, 1)
        data = frame()
        data["right"]["p"][0] = float("nan")
        with self.assertRaises(ValueError):
            decode_message(json.dumps(data), 1)

    def test_native_event_preserves_device_time_and_reference_change(self) -> None:
        event = decode_message(reference_event(change_time=99), 456)
        self.assertIsInstance(event, Event)
        self.assertEqual(event.observed_monotonic_ns, 456)
        self.assertIn("reference", event.kind)
        self.assertIn("123", json.dumps(dict(event.details)))
        self.assertNotEqual(sample(origin=0).header.ref.epoch, sample(origin=1).header.ref.epoch)


class QuestUsbTests(unittest.TestCase):
    def test_source_readiness_follows_monitor_publication(self) -> None:
        source, monitor = QuestSource(), QuestInputMonitor()
        source._session, source._running = "test-session", True
        ready_during_publish = []

        def publish(value):
            ready_during_publish.append(source.health().ready)
            return monitor.try_publish(value)

        source._sink = Mock(try_publish=publish)
        with patch("bimanual_teleop.devices.quest.adapter.time.monotonic_ns", return_value=1_000_000_000):
            source._process_line(json.dumps(frame(0, refresh_hz=90)), 1_000_000_000)
            self.assertEqual(ready_during_publish, [False])
            self.assertTrue(source.health().ready)
            self.assertIs(monitor.current(1_000_000_000)[0], source.get_latest())

    def test_small_source_gap_preserves_fresh_input_without_permanent_source_fault(self) -> None:
        source, monitor = QuestSource(), QuestInputMonitor()
        source._session, source._sink, source._running = "test-session", monitor, True
        source._process_line(json.dumps(frame(0, refresh_hz=90)), 1_000_000_000)
        source._process_line(json.dumps(frame(3, refresh_hz=90)), 1_030_000_000)
        with patch("bimanual_teleop.devices.quest.adapter.time.monotonic_ns", return_value=1_030_000_000):
            self.assertTrue(source.health().ready)
            self.assertEqual(monitor.current(1_030_000_000, check_latch=True)[0].payload.sequence, 3)
        # Skipped samples do not grant extra time to reuse the current sample.
        with self.assertRaisesRegex(RuntimeError, "silent or additionally queued"):
            monitor.current(1_130_000_000, check_latch=True)

    def test_clean_frames_recover_data_health_but_do_not_clear_control_fault_latch(self) -> None:
        for kind in ("malformed", "out_of_order", "reference_space_change", "reference_metadata_gap"):
            with self.subTest(kind=kind):
                source, monitor = QuestSource(), QuestInputMonitor()
                source._session, source._sink, source._running = "test-session", monitor, True
                source._process_line(json.dumps(frame(0, refresh_hz=90)), 1_000_000_000)
                origin, sequence = 0, 1
                if kind == "malformed":
                    line = '{"v":'
                elif kind == "out_of_order":
                    line = json.dumps(frame(0, refresh_hz=90))
                elif kind == "reference_space_change":
                    line, origin = reference_event(), 1
                else:
                    line = json.dumps(frame(1, origin=1, refresh_hz=90))
                    origin, sequence = 1, 2
                source._process_line(line, 1_005_000_000)
                with patch("bimanual_teleop.devices.quest.adapter.time.monotonic_ns", return_value=1_005_000_000):
                    self.assertFalse(source.health().ready)
                now = 1_000_000_000 + sequence * 10_000_000
                source._process_line(json.dumps(frame(sequence, origin=origin, refresh_hz=90)), now)
                with patch("bimanual_teleop.devices.quest.adapter.time.monotonic_ns", return_value=now):
                    self.assertTrue(source.health().ready)
                with self.assertRaises(RuntimeError):
                    monitor.current(now, check_latch=True)
                self.assertIsNotNone(monitor.fault)
                monitor.current(now, acknowledge=True)
                self.assertIs(monitor.current(now, check_latch=True)[0], source.get_latest())

    def test_new_frame_cannot_clear_disconnection_or_runtime_fault(self) -> None:
        for kind in ("disconnected", "error"):
            with self.subTest(kind=kind):
                source = QuestSource()
                source._session, source._running = "test-session", True
                source._issue(kind, "persistent source failure", 1_000_000_000)
                source._process_line(json.dumps(frame(0, refresh_hz=90)), 1_000_000_000)
                with patch("bimanual_teleop.devices.quest.adapter.time.monotonic_ns", return_value=1_000_000_000):
                    self.assertFalse(source.health().ready)
                    self.assertEqual(source.health().detail, "persistent source failure")

    def test_single_side_health_checks_only_requested_tracking(self):
        source = QuestSource()
        source._session, source._running = "test-session", True
        data = frame(0, refresh_hz=90)
        for name in ("head", "right"):
            data[name].update(p=None, q=None, flags=0)
        source._process_line(json.dumps(data), 42)
        with patch("bimanual_teleop.devices.quest.adapter.time.monotonic_ns", return_value=42):
            self.assertTrue(source.health(sides=("left",), require_head=False).ready)
            self.assertFalse(source.health(sides=("left",)).ready)
            self.assertFalse(source.health(sides=("right",), require_head=False).ready)
            self.assertFalse(source.health().ready)

    def test_both_controller_health_can_ignore_head_tracking(self):
        source = QuestSource()
        source._session, source._running = "test-session", True
        data = frame(0, refresh_hz=90)
        data["head"].update(p=None, q=None, flags=0)
        source._process_line(json.dumps(data), 42)
        with patch("bimanual_teleop.devices.quest.adapter.time.monotonic_ns", return_value=42):
            self.assertTrue(source.health(require_head=False).ready)
            self.assertIn("head tracking unavailable", source.health().detail)

    def test_health_identifies_untracked_controller_separately_from_focus(self):
        source = QuestSource()
        source._session, source._running = "test-session", True
        data = frame(0, refresh_hz=90)
        data["right"].update(p=None, q=None, flags=0)
        source._process_line(json.dumps(data), 42)
        with patch("bimanual_teleop.devices.quest.adapter.time.monotonic_ns", return_value=42):
            self.assertIn("right tracking unavailable", source.health().detail)
            self.assertNotIn("not focused", source.health().detail)
        source._process_line(json.dumps(frame(1, refresh_hz=90)), 43)
        with patch("bimanual_teleop.devices.quest.adapter.time.monotonic_ns", return_value=43):
            self.assertTrue(source.health().ready)

    def test_latest_reads_do_not_publish_or_restamp_and_old_sessions_are_ignored(self) -> None:
        source, sink = QuestSource(), Sink()
        source._session, source._sink, source._running = "test-session", sink, True
        source._process_line(json.dumps(frame(0)), 42)
        first = source.get_latest()
        self.assertIs(source.get_latest(), first)
        self.assertEqual(first.header.received_monotonic_ns, 42)
        source._process_line(json.dumps(frame(10, session="previous-launch")), 99)
        self.assertIs(source.get_latest(), first)
        self.assertEqual(len(sink.samples), 1)
        self.assertEqual(sink.events, [])

    def test_sequence_gaps_duplicates_and_truncation_are_visible(self) -> None:
        source, sink = QuestSource(), Sink()
        source._session, source._sink, source._running = "test-session", sink, True
        source._process_line(json.dumps(frame(0)), 42)
        source._process_line(json.dumps(frame(3)), 43)
        gap = next(event for event in sink.events if event.kind == "quest.source_gap")
        self.assertEqual(gap.details["first_source_sequence"], 1)
        self.assertEqual(gap.details["last_source_sequence"], 2)
        source._process_line(json.dumps(frame(3)), 44)
        source._process_line(json.dumps(frame(4))[:-10], 45)
        self.assertEqual(len(sink.samples), 2)
        self.assertEqual(source.get_latest().header.source_sequence, 3)
        self.assertTrue(any(event.kind == "quest.out_of_order" for event in sink.events))
        self.assertTrue(any(event.kind == "quest.malformed" for event in sink.events))
        self.assertFalse(source.health().ready)

    def test_observer_rejection_or_exception_stops_live_tracking(self) -> None:
        for error in (None, RuntimeError("observer offline")):
            with self.subTest(error=error):
                source, sink = QuestSource(), Sink()
                source._session, source._sink, source._running = "test-session", sink, True
                sink.try_publish = Mock(return_value=False, side_effect=error)
                with self.assertLogs("bimanual_teleop.devices.quest.adapter", level="ERROR") as logs:
                    for sequence in range(3):
                        source._process_line(json.dumps(frame(sequence, refresh_hz=90)), 42 + sequence)
                self.assertEqual(len(logs.records), 1)
                self.assertFalse(sink.events)
                self.assertIn("observer offline" if error else "rejected", source.metadata["observer_error"])
                with patch("bimanual_teleop.devices.quest.adapter.time.monotonic_ns", return_value=44):
                    self.assertFalse(source.health().ready)
                self.assertEqual(source.get_latest().payload.sequence, 2)
                # A separate source failure remains observable in the same session.
                source._process_line("{malformed", 45)
                self.assertFalse(source.health().ready)

    def test_event_observer_failure_invalidates_source_health(self) -> None:
        source, sink = QuestSource(), Sink()
        source._session, source._sink, source._running = "test-session", sink, True
        sink.try_event = Mock(return_value=False)
        with self.assertLogs("bimanual_teleop.devices.quest.adapter", level="ERROR"):
            source._event(Event("quest.metadata", 41, "quest"))
        source._process_line(json.dumps(frame(0, refresh_hz=90)), 42)
        with patch("bimanual_teleop.devices.quest.adapter.time.monotonic_ns", return_value=42):
            self.assertFalse(source.health().ready)
        self.assertIn("rejected", source.metadata["observer_error"])

    def test_repeated_source_warning_is_rate_limited_without_losing_events(self) -> None:
        source, sink = QuestSource(), Sink()
        source._session, source._sink, source._running = "test-session", sink, True
        with self.assertLogs("bimanual_teleop.devices.quest.adapter", level="WARNING") as logs:
            source._issue("malformed", "bad frame", 1_000_000_000)
            source._issue("malformed", "bad frame", 1_000_000_001)
            source._issue("malformed", "bad frame", 6_000_000_000)
        self.assertEqual(len(logs.records), 2)
        self.assertEqual([event.kind for event in sink.events], ["quest.malformed"] * 3)

    def test_reference_change_invalidates_latest_until_new_origin(self) -> None:
        source, sink = QuestSource(), Sink()
        source._session, source._sink, source._running = "test-session", sink, True
        source._process_line(json.dumps(frame(0, refresh_hz=90)), 42)
        previous_epoch = source.get_latest().header.ref.epoch
        source._process_line(reference_event(), 43)
        self.assertIsNone(source.get_latest())
        self.assertFalse(source.health().ready)
        self.assertIn("reference space is changing", source.health().detail)
        self.assertNotIn("waiting for Quest frames", source.health().detail)
        source._process_line(json.dumps(frame(1, origin=1, refresh_hz=90)), 44)
        self.assertNotEqual(source.get_latest().header.ref.epoch, previous_epoch)
        with patch("bimanual_teleop.devices.quest.adapter.time.monotonic_ns", return_value=44):
            self.assertTrue(source.health().ready)

    def test_origin_change_without_event_keeps_data_but_marks_missing_metadata(self) -> None:
        source, sink = QuestSource(), Sink()
        source._session, source._sink, source._running = "test-session", sink, True
        source._process_line(json.dumps(frame(0, refresh_hz=90)), 42)
        source._process_line(json.dumps(frame(1, origin=1, refresh_hz=90)), 43)
        self.assertEqual(len(sink.samples), 2)
        self.assertTrue(any(event.kind == "quest.origin_changed" for event in sink.events))
        self.assertTrue(any(event.kind == "quest.reference_metadata_gap" for event in sink.events))
        self.assertFalse(source.health().ready)

    def test_origin_ids_need_not_follow_numeric_order(self) -> None:
        source, sink = QuestSource(), Sink()
        source._session, source._sink, source._running = "test-session", sink, True
        source._process_line(json.dumps(frame(0, refresh_hz=90)), 42)
        source._process_line(reference_event(2, 2_010_000_000), 43)
        source._process_line(reference_event(1, 2_020_000_000), 44)
        source._process_line(json.dumps(frame(1, origin=2, refresh_hz=90)), 45)
        source._process_line(json.dumps(frame(2, origin=1, refresh_hz=90)), 46)
        self.assertEqual([item.payload.origin for item in sink.samples], [0, 2, 1])
        with patch("bimanual_teleop.devices.quest.adapter.time.monotonic_ns", return_value=46):
            self.assertTrue(source.health().ready)
        self.assertFalse(any(event.kind == "quest.reference_metadata_gap" for event in sink.events))

    def test_host_arrival_silence_is_reported_and_recovers_on_a_new_frame(self) -> None:
        source = QuestSource()
        source._session, source._running = "test-session", True
        source._process_line(json.dumps(frame(0, refresh_hz=90)), 1_000_000_000)
        with patch("bimanual_teleop.devices.quest.adapter.time.monotonic_ns", return_value=1_000_000_000):
            self.assertTrue(source.health().ready)
        with patch("bimanual_teleop.devices.quest.adapter.time.monotonic_ns", return_value=2_100_000_000):
            self.assertFalse(source.health().ready)
        source._process_line(json.dumps(frame(1, refresh_hz=90)), 2_100_000_001)
        with patch("bimanual_teleop.devices.quest.adapter.time.monotonic_ns", return_value=2_100_000_002):
            self.assertTrue(source.health().ready)

    def test_malformed_event_is_reported_without_losing_the_previous_sample(self) -> None:
        source, sink = QuestSource(), Sink()
        source._session, source._sink, source._running = "test-session", sink, True
        source._process_line(json.dumps(frame(0, refresh_hz=90)), 42)
        previous = source.get_latest()
        for kind, details in [
            ("session_state", {"state": 9, "time": 99}),
            ("refresh_rate", {"from_hz": 90, "to_hz": float("nan")}),
            ("reference_space_change", {"origin": 1, "change_time": 99}),
        ]:
            with self.subTest(kind=kind):
                line = json.dumps({"v": 1, "type": "event", "session": "test-session",
                                   "event": kind, "device_ns": 43, "details": details})
                source._process_line(line, 44)
                self.assertEqual(sink.events[-1].kind, "quest.malformed")
                self.assertIs(source.get_latest(), previous)

    def test_start_and_usb_eof_use_mock_adb_and_stop_only_owned_app(self) -> None:
        listing = "List of devices attached\nusb-a device usb:1-2 model:Quest_3\n"
        process = Mock(stdout=io.StringIO(json.dumps(frame(0, refresh_hz=90)) + "\n"))
        process.wait.return_value = 0
        process.poll.return_value = None

        def adb(command, **kwargs):
            if command[1:] == ["devices", "-l"]:
                output = listing
            elif "pidof" in command:
                raise subprocess.CalledProcessError(1, command)
            elif "pm" in command:
                output = "package:/data/app/quest/base.apk\n"
            else:
                output = ""
            return subprocess.CompletedProcess(command, 0, output, "")

        with patch("bimanual_teleop.devices.quest.adapter.subprocess.run", side_effect=adb) as run, \
             patch("bimanual_teleop.devices.quest.adapter.subprocess.Popen", return_value=process) as popen, \
             patch("bimanual_teleop.devices.quest.adapter.uuid.uuid4", return_value=Mock(hex="test-session")):
            source, sink = QuestSource(), Sink()
            source.start(sink)
            source._thread.join(timeout=2)
            self.assertEqual(len(sink.samples), 1)
            self.assertFalse(source.health().ready)
            self.assertTrue(any(event.kind == "quest.disconnected" for event in sink.events))
            source.close()
            process.terminate.assert_called_once()
            self.assertIn("usb-a", popen.call_args.args[0])
            self.assertTrue(any("force-stop" in call.args[0] for call in run.call_args_list))
            source.close()
            self.assertEqual(sum("force-stop" in call.args[0] for call in run.call_args_list), 1)

    def test_adb_eof_retains_transport_diagnostic_and_exit_code(self) -> None:
        source, sink = QuestSource(), Sink()
        source._session, source._sink, source._running = "test-session", sink, True
        source._process = Mock(stdout=io.StringIO("adb: error: device offline\n"))
        source._process.poll.return_value = 1
        source._read_loop()
        event = next(event for event in sink.events if event.kind == "quest.disconnected")
        self.assertEqual(event.details["exit_code"], 1)
        self.assertEqual(event.details["transport_diagnostic"], "adb: error: device offline")
        self.assertIn("device offline", source.health().detail)
        self.assertIn("exit code 1", source.health().detail)

    def test_expected_shutdown_does_not_report_adb_disconnection(self) -> None:
        source, sink = QuestSource(), Sink()
        source._session, source._sink = "test-session", sink
        source._process = Mock(stdout=io.StringIO(""))
        source._read_loop()
        self.assertFalse(sink.events)

    def test_usb_selection_excludes_network_devices_and_requires_unambiguous_quest(self) -> None:
        listing = """List of devices attached
usb-a device usb:1-2 product:panther model:Quest_3 device:panther transport_id:1
192.168.1.2:5555 device product:panther model:Quest_3 device:panther transport_id:2
phone device usb:1-3 product:pixel model:Pixel_8 device:shiba transport_id:3
"""
        self.assertEqual(select_usb_serial(listing), "usb-a")
        two = listing + "usb-b device usb:1-4 product:reindeer model:Quest_3S device:reindeer transport_id:4\n"
        with self.assertRaises((ValueError, RuntimeError)):
            select_usb_serial(two)
        self.assertEqual(select_usb_serial(two, "usb-b"), "usb-b")
        for requested in ["192.168.1.2:5555", "missing", "phone"]:
            with self.subTest(requested=requested), self.assertRaises((ValueError, RuntimeError)):
                select_usb_serial(two, requested)
        with self.assertRaises((ValueError, RuntimeError)):
            select_usb_serial("List of devices attached\nusb-a unauthorized usb:1-2\n")


if __name__ == "__main__":
    unittest.main()
