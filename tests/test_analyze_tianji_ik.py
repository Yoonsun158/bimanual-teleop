"""Offline diagnostic replay against the actual kinematics SDK, without devices."""

import ast
from contextlib import redirect_stderr, redirect_stdout
from copy import deepcopy
from dataclasses import asdict
import io
import json
import math
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest
from unittest.mock import patch

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from scripts import analyze_tianji_ik as analysis  # noqa: E402


def diagnostic():
    return {
        "schema": analysis.SCHEMA, "sdk_commit": analysis.SDK_COMMIT,
        "model_sha256": analysis.MODEL_SHA256, "side": "left",
        "target": {"parent_frame": "tianji_left_base", "child_frame": "tianji_left_flange",
                   "position_m": [0.5, 0.1, 0.2], "orientation_xyzw": [0, 0, 0, 1]},
        "reference_deg": [30, -60, -34, -52, 30, 12, 4], "result_deg": [0] * 7,
        "status": 0, "solution_count": 1, "out_of_range": 0,
        "singular_mask": 0, "limit_mask": 0,
    }


class DiagnosticInputTests(unittest.TestCase):
    def test_json_objects_arrays_and_marked_lines_with_ui_suffix(self):
        first, second = diagnostic(), diagnostic()
        second["reference_deg"][0] = 31
        self.assertEqual(analysis.read_records(json.dumps(first))[0]["diagnostic"], first)
        self.assertEqual(len(analysis.read_records(json.dumps([first, second]))), 2)
        text = ("[提示] 正在接合\n[IK诊断] " + json.dumps(first) +
                "；恢复时按 Enter 或双手重新比 V。\n[IK诊断] " + json.dumps(second) + "\n[完成] 已退出。")
        records = analysis.read_records(text)
        self.assertEqual([row["diagnostic"] for row in records], [first, second])
        self.assertEqual([row["line"] for row in records], [2, 3])
        self.assertEqual(len(analysis.read_records(json.dumps(first) + "\n" + json.dumps(second))), 2)

    def test_broken_record_does_not_hide_later_records(self):
        records = analysis.read_records("[IK诊断] {broken\n[IK诊断] " + json.dumps(diagnostic()))
        self.assertIn("invalid diagnostic JSON", records[0]["error"])
        self.assertEqual(records[1]["diagnostic"]["schema"], analysis.SCHEMA)
        with self.assertRaisesRegex(ValueError, "no JSON diagnostics"):
            analysis.read_records("[警告] IK failed without diagnostic values")

    def test_missing_fields_versions_and_nonfinite_inputs_are_rejected(self):
        for key in diagnostic():
            value = diagnostic()
            value.pop(key)
            with self.subTest(missing=key), self.assertRaisesRegex(ValueError, "missing"):
                analysis.validate_diagnostic(value)
        for key, replacement, reason in (
                ("schema", "v9", "schema"), ("sdk_commit", "wrong", "SDK commit mismatch"),
                ("model_sha256", "wrong", "model SHA256 mismatch"), ("side", "both", "side"),
                ("reference_deg", [math.nan] * 7, "finite"),
                ("reference_deg", [10**1000] * 7, "finite"),
                ("reference_deg", [True] * 7, "finite"),
                ("reference_deg", [0] * 6, "7 numbers"),
                ("status", False, "integer"), ("status", None, "initialization"),
                ("solution_count", -1, "nonnegative"),
                ("out_of_range", 3, "boolean"), ("limit_mask", 128, "seven joints")):
            value = diagnostic()
            value[key] = replacement
            with self.subTest(field=key), self.assertRaisesRegex(ValueError, reason):
                analysis.validate_diagnostic(value)
        for field, replacement in (("position_m", [math.inf, 0, 0]),
                                   ("orientation_xyzw", [1, 1, 1, 1]),
                                   ("parent_frame", "world")):
            value = diagnostic()
            value["target"][field] = replacement
            with self.subTest(target=field), self.assertRaises((ValueError, RuntimeError)):
                analysis.validate_diagnostic(value)

    def test_unknown_output_joints_and_additional_timing_fields_are_preserved(self):
        value = diagnostic()
        value.update(result_deg=[None] + [0] * 6, command_id="test",
                     feedback={"received_monotonic_ns": 123, "joints_deg": [0] * 7})
        self.assertIsNone(analysis.validate_diagnostic(value)["result_deg"][0])
        self.assertEqual(analysis.validate_diagnostic(value)["feedback"], value["feedback"])
        value["result_deg"] = None
        self.assertIsNone(analysis.validate_diagnostic(value)["result_deg"])

    def test_bad_input_cli_never_loads_a_library(self):
        value = diagnostic()
        value["sdk_commit"] = "mismatch"
        stdout = io.StringIO()
        with patch.object(sys, "stdin", io.StringIO(json.dumps(value))), redirect_stdout(stdout), \
                patch("ctypes.CDLL", side_effect=AssertionError("must not load")):
            self.assertEqual(analysis.main(["-"]), 1)
        self.assertIn("SDK commit mismatch", json.loads(stdout.getvalue())["error"])

    def test_control_bridge_path_is_rejected_before_load(self):
        with patch("ctypes.CDLL", side_effect=AssertionError("must not load")):
            with self.assertRaisesRegex(RuntimeError, "SDK file missing") :
                analysis.OfflineKinematics(analysis.DEFAULT_MODEL.parent.parent / "SDK_PYTHON/libMarvinSDK.so")

    def test_import_has_no_native_load_or_transport_dependencies(self):
        code = """
import ctypes, sys
from unittest.mock import patch
with patch.object(ctypes, 'CDLL', side_effect=AssertionError('native load on import')):
    from scripts import analyze_tianji_ik
assert 'bimanual_teleop.devices.tianji.driver' not in sys.modules
assert 'bimanual_teleop.devices.quest.adapter' not in sys.modules
"""
        subprocess.run([sys.executable, "-c", code], check=True, cwd=ROOT,
                       capture_output=True, text=True, timeout=10)
        source = Path(analysis.__file__).read_text()
        tree = ast.parse(source)
        imports = {node.module or "" for node in ast.walk(tree) if isinstance(node, ast.ImportFrom)}
        imports.update(alias.name for node in ast.walk(tree) if isinstance(node, ast.Import) for alias in node.names)
        self.assertFalse(imports & {"socket", "subprocess", "bimanual_teleop.devices.tianji.driver",
                                   "bimanual_teleop.devices.quest.adapter"})
        self.assertNotIn("tj_ik_continuous", source)


class NativeDiagnosticReplayTests(unittest.TestCase):
    def setUp(self):
        self.kine = analysis.OfflineKinematics()

    def capture(self, side="left", q=(30, -60, -34, -52, 30, 12, 4)):
        value = diagnostic()
        value.update(side=side, target=asdict(self.kine.fk(side, q)), reference_deg=q)
        replay = self.kine._solve(value)
        for key in ("result_deg", "status", "solution_count", "out_of_range", "singular_mask", "limit_mask"):
            value[key] = replay[key]
        return value

    def test_real_fk_target_replays_exactly_and_reports_fixed_direction_jumps(self):
        for side in ("left", "right"):
            with self.subTest(side=side):
                value = self.capture(side)
                unchanged = deepcopy(value)
                report = self.kine.analyze(value)
                self.assertEqual(value, unchanged)
                self.assertTrue(report["replay_matches_captured_flags"])
                self.assertEqual(report["replay_result_max_difference_deg"], 0)
                baseline, pico, htc = report["candidates"]
                self.assertTrue(baseline["legal_single_pose"])
                self.assertLess(baseline["fk_position_error_m"], 1e-6)
                self.assertLess(baseline["fk_orientation_error_deg"], 0.001)
                self.assertEqual([item["method"] for item in report["candidates"]],
                                 ["NEAR_REF", "NEAR_DIR_PICO", "NEAR_DIR_HTC"])
                self.assertEqual(baseline["singularity_tolerance_deg"], [0, 0, 0])
                self.assertEqual(pico["singularity_tolerance_deg"], [5, 5, 5])
                self.assertIn("tianji_chest_driver.py", htc["source"])
                self.assertGreater(pico["max_joint_delta_deg"], 10)
                self.assertGreater(htc["max_joint_delta_deg"], 10)
                self.assertIn("不保证轨迹连续", report["limitation"])
                self.assertNotIn("nsp_scan", report)

    def test_j6_limit_failure_has_legal_offline_alternatives_with_same_fk_pose(self):
        value = self.capture(q=(30, -60, -34, -52, 30, -60.23, 4))
        self.assertTrue(value["limit_mask"] & 32)
        report = self.kine.analyze(value, scan_nsp=True)
        self.assertFalse(report["candidates"][0]["legal_single_pose"])
        self.assertTrue(report["replay_matches_captured_flags"])
        self.assertGreater(report["nsp_scan"]["legal_count"], 0)
        for candidate in report["nsp_scan"]["nearest_legal_candidates"]:
            self.assertTrue(candidate["legal_single_pose"])
            self.assertEqual(candidate["limit_mask"], 0)
            self.assertFalse(candidate["effective_limit_violations"])
            self.assertLess(candidate["fk_position_error_m"], 1e-6)
            self.assertGreater(candidate["max_joint_delta_deg"], 1)

    def test_unknown_captured_result_does_not_claim_a_replay_joint_match(self):
        value = self.capture()
        value["result_deg"] = [None] + list(value["result_deg"])[1:]
        self.assertIsNone(self.kine.analyze(value)["replay_result_max_difference_deg"])

    def test_unreachable_target_and_invalid_scan_step_are_visible(self):
        value = self.capture()
        value["target"]["position_m"] = [10, 10, 10]
        report = self.kine.analyze(value, scan_nsp=True, scan_step_deg=90)
        self.assertFalse(any(row["legal_single_pose"] for row in report["candidates"]))
        self.assertEqual(report["nsp_scan"]["legal_count"], 0)
        with self.assertRaisesRegex(ValueError, "scan_step_deg"):
            self.kine.analyze(value, scan_nsp=True, scan_step_deg=0)

    def test_cli_handles_all_diagnostics_and_returns_error_for_the_bad_record(self):
        good = self.capture()
        bad = {**good, "model_sha256": "wrong"}
        text = "[IK诊断] " + json.dumps(good) + "；恢复时按 Enter\n"
        text += "[IK诊断] " + json.dumps(bad) + "\n[IK诊断] " + json.dumps(good)
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "console.log"
            path.write_text(text)
            stdout, stderr = io.StringIO(), io.StringIO()
            with redirect_stdout(stdout), redirect_stderr(stderr):
                self.assertEqual(analysis.main([str(path)]), 1)
        records = [json.loads(line) for line in stdout.getvalue().splitlines()]
        self.assertEqual([row["record"] for row in records], [1, 2, 3])
        self.assertTrue(records[0]["replay_matches_captured_flags"])
        self.assertIn("model SHA256 mismatch", records[1]["error"])
        self.assertTrue(records[2]["replay_matches_captured_flags"])
        self.assertEqual(stderr.getvalue(), "")

    def test_fresh_process_loads_only_kinematics_library(self):
        code = """
import sys
from pathlib import Path
from scripts.analyze_tianji_ik import OfflineKinematics
kine = OfflineKinematics()
kine.fk('left', (30,-60,-34,-52,30,12,4))
assert 'bimanual_teleop.devices.tianji.driver' not in sys.modules
assert 'bimanual_teleop.devices.quest.adapter' not in sys.modules
maps = Path('/proc/self/maps').read_text()
assert 'libKine.so' in maps
assert 'libMarvinSDK.so' not in maps
"""
        subprocess.run([sys.executable, "-c", code], check=True, cwd=ROOT,
                       capture_output=True, text=True, timeout=10)


if __name__ == "__main__":
    unittest.main()
