"""Stop-cause propagation and timing attribution; all devices are synthetic."""

import json
import unittest
from unittest.mock import patch

from bimanual_teleop.cli.runtime import TeleopUI
from tests.support.quest import RuntimeFixture


class PauseDiagnosticTests(unittest.TestCase):
    def setUp(self):
        self.fx = RuntimeFixture()
        self.addCleanup(self.fx.runtime.close)
        clock = patch("bimanual_teleop.control.arm.cartesian.time.monotonic_ns",
                      side_effect=self.fx.clock)
        clock.start()
        self.addCleanup(clock.stop)
        self.fx.runtime.engage()
        self.fx.runtime.tick()

    def stop_driver(self, reason):
        self.fx.driver.motion_stop = {"reason": reason, "monotonic_ns": self.fx.clock(),
                                      "target_age_ms": 51.}
        self.fx.driver.engaged = False

    def test_watchdog_during_planning_reports_original_reason_and_slow_side(self):
        fx = self.fx
        fx.advance()
        calls = len(fx.driver.commands)

        def delayed_left_solve():
            fx.kine.after_ik = None
            fx.clock.advance(46_000_000)
            self.stop_driver("Accepted target expired within host watchdog interval")

        fx.kine.after_ik = delayed_left_solve
        self.assertIsNone(fx.runtime.tick())
        self.assertEqual(len(fx.driver.commands), calls)
        self.assertEqual(fx.runtime.last_error, fx.driver.motion_stop["reason"])
        diagnostic = fx.runtime.last_pause_diagnostic
        self.assertEqual(diagnostic["executor"]["left_servo_ns"], 46_000_000)
        self.assertEqual(diagnostic["executor"]["right_servo_ns"], 0)
        self.assertEqual(diagnostic["cycle"]["executor_ns"], 46_000_000)
        self.assertEqual(diagnostic["cycle"]["source_age_ns"], 0)
        self.assertEqual(diagnostic["driver_stop"], fx.driver.motion_stop)
        # A coordinator's repeated pause must not rewrite the first evidence.
        fx.runtime.pause("coordinator cleanup")
        self.assertIs(fx.runtime.last_pause_diagnostic, diagnostic)

    def test_stop_between_cycles_preserves_driver_reason(self):
        self.fx.advance()
        self.stop_driver("left source counter has not advanced within the host watchdog interval")
        self.fx.runtime.tick()
        self.assertEqual(self.fx.runtime.last_error, self.fx.driver.motion_stop["reason"])
        self.assertEqual(self.fx.runtime.last_pause_diagnostic["cycle"]["tick_gap_ns"], 5_000_000)

    def test_recent_receipt_does_not_hide_short_source_deadline(self):
        fx = self.fx
        query = fx.quest.latest.payload.query_monotonic_ns
        fx.clock.advance(99_000_000)
        fx.driver.emit()
        fx.quest.emit(query_ns=query + 11_111_111)
        self.assertIsNotNone(fx.runtime.tick())
        timing = fx.runtime.cycle_timing
        self.assertEqual(timing["source_age_ns"], 0)
        self.assertEqual(timing["input_remaining_ns"], 12_111_111)
        self.assertEqual(timing["target_remaining_ns"], 12_111_111)

    def test_ui_emits_one_machine_readable_snapshot_then_clears_on_engagement(self):
        self.stop_driver("Previous target watchdog expired; re-engage before submitting")
        self.fx.runtime.tick()
        messages = []
        ui = TeleopUI(self.fx.runtime, self.fx.profile, emit=messages.append, verbose=True)
        ui.loop_timing = {"last_status_ns": 81_000_000, "pre_tick_ns": 10_000}
        ui.report_runtime_pause()
        ui.report_runtime_pause()
        rows = [line for line in messages if line.startswith("[停机诊断] ")]
        self.assertEqual(len(rows), 1)
        payload = json.loads(rows[0].split(" ", 1)[1])
        self.assertEqual(payload["host_loop"]["last_status_ns"], 81_000_000)
        self.assertEqual(payload["driver_stop"]["reason"], self.fx.runtime.last_error)
        self.fx.advance()
        self.fx.runtime.engage()
        self.assertIsNone(self.fx.runtime.last_pause_diagnostic)

    def test_default_output_keeps_pause_reason_without_snapshot(self):
        self.stop_driver("Previous target watchdog expired; re-engage before submitting")
        self.fx.runtime.tick()
        messages = []
        ui = TeleopUI(self.fx.runtime, self.fx.profile, emit=messages.append)
        ui.report_runtime_pause()
        ui.report_runtime_pause()
        self.assertEqual(len(messages), 1)
        self.assertIn(self.fx.runtime.last_error, messages[0])
        self.assertIn("恢复时按 Enter", messages[0])
        self.assertNotIn("[停机诊断]", messages[0])
        self.assertIsNotNone(self.fx.runtime.last_pause_diagnostic)


if __name__ == "__main__":
    unittest.main()
