"""Ready-pose sequencing and no-runtime-record checks without hardware."""

from contextlib import redirect_stderr
from dataclasses import dataclass, field
import io
import json
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

from bimanual_teleop.cli import prepare_tianji_teleop as ready

ROOT = Path(__file__).resolve().parents[1]


@dataclass
class Arm:
    position_rad: tuple = (0.,) * 7
    state: int = 1
    error: int = 0

    @property
    def joints(self):
        return self


@dataclass
class Feedback:
    payload: object = field(default_factory=lambda: type("Frame", (), {
        "arms": {side: Arm() for side in ready.SIDES}})())


class Driver:
    def __init__(self):
        self.calls = []
        self.sample = Feedback()
        self.failure = self.close_error = None

    def start(self):
        self.calls.append("start")

    def configure(self, profile):
        self.calls.append("configure")
        self.profile = profile

    def get_latest(self):
        return self.sample

    def move_joints(self, side, target):
        self.calls.append((side, target))
        if self.failure == side:
            raise RuntimeError("SDK rejected joint target")
        self.sample.payload.arms[side].position_rad = target
        return self.sample

    def reset_released_emergency(self, *, physical_release_confirmed):
        assert physical_release_confirmed
        self.calls.append("reset")
        for arm in self.sample.payload.arms.values():
            arm.error = 0
        return {"confirmed_disabled": True}

    def close(self):
        self.calls.append("close")
        if self.close_error:
            raise RuntimeError(self.close_error)


class ReadyPoseTests(unittest.TestCase):
    def setUp(self):
        temporary = tempfile.TemporaryDirectory()
        self.addCleanup(temporary.cleanup)
        self.root = Path(temporary.name)
        self.target = self.root / "config.json"
        self.settings = ready.load_config(ROOT / "configs/tianji_teleop.json")
        self.source = self.settings["ready_pose"]
        self.driver = Driver()

    def invoke(self, *flags):
        self.target.write_text(json.dumps(self.settings))
        before = {path.relative_to(self.root) for path in self.root.rglob("*") if path.is_file()}
        args = ready.parser().parse_args(["--tianji-config", str(self.target), *flags])
        output = io.StringIO()
        with patch.object(ready, "TianjiDriver", return_value=self.driver) as factory, \
             patch.object(ready, "configure_runtime_logging"), redirect_stderr(output):
            code = ready.run(args)
        after = {path.relative_to(self.root) for path in self.root.rglob("*") if path.is_file()}
        self.assertEqual(after, before)
        return code, output.getvalue(), factory

    def test_preview_does_not_connect_or_write(self):
        code, output, factory = self.invoke()
        self.assertEqual(code, 0)
        factory.assert_not_called()
        self.assertIn("初始关节目标已加载", output)

    def test_inspect_reads_without_motion(self):
        code, output, factory = self.invoke("--inspect")
        self.assertEqual(code, 0)
        factory.assert_called_once()
        self.assertEqual(self.driver.calls, ["start", "close"])
        self.assertIn("状态", output)

    def test_inspect_uses_shared_config_without_manual_ip(self):
        self.settings["controller_ip"] = "192.0.2.8"
        code, _, factory = self.invoke("--inspect")
        self.assertEqual(code, 0)
        factory.assert_called_once_with("192.0.2.8", None, model_path=None)

    def test_ip_override_preserves_ready_pose_from_shared_config(self):
        code, _, factory = self.invoke("--execute", "--ip", "192.0.2.9")
        self.assertEqual(code, 0)
        factory.assert_called_once_with("192.0.2.9", None, model_path=None)
        self.assertEqual(self.driver.profile.parameters["arms"]["left"]["velocity_ratio"], 25)

    def test_enable_motion_uses_selected_target_and_custom_model(self):
        library, model = self.root / "custom.so", self.root / "model.MvKDCfg"
        code, _, factory = self.invoke("--enable-motion", "--side", "right",
                                       "--library", str(library), "--model", str(model))
        self.assertEqual(code, 0)
        factory.assert_called_once_with(self.settings["controller_ip"], library, model_path=model)
        moves = [call for call in self.driver.calls if isinstance(call, tuple)]
        self.assertEqual(len(moves), 1)
        self.assertEqual(moves[0][0], "right")
        for actual, expected in zip(moves[0][1], self.source["target_deg"]["right"]):
            self.assertAlmostEqual(ready.math.degrees(actual), expected)

    def test_execute_moves_selected_arms_in_order_and_closes(self):
        self.source["order"] = ["right", "left"]
        code, output, _ = self.invoke("--execute")
        self.assertEqual(code, 0)
        self.assertEqual(self.driver.calls[0:2], ["start", "configure"])
        self.assertEqual([call[0] for call in self.driver.calls if isinstance(call, tuple)], ["right", "left"])
        self.assertEqual(self.driver.calls[-1], "close")
        self.assertIn("准备完成", output)

    def test_single_side_does_not_require_other_arm_target(self):
        self.source["target_deg"] = {"left": self.source["target_deg"]["left"]}
        code, _, _ = self.invoke("--execute", "--side", "left")
        self.assertEqual(code, 0)
        self.assertEqual(self.driver.profile.parameters["active_arms"], ["left"])
        self.assertEqual([call[0] for call in self.driver.calls if isinstance(call, tuple)], ["left"])

    def test_failure_blocks_next_arm_and_reports_error_without_record(self):
        self.driver.failure = "left"
        code, output, _ = self.invoke("--execute")
        self.assertEqual(code, 1)
        self.assertEqual([call[0] for call in self.driver.calls if isinstance(call, tuple)], ["left"])
        self.assertEqual(self.driver.calls[-1], "close")
        self.assertIn("SDK rejected", output)

    def test_close_failure_changes_exit_code(self):
        self.driver.close_error = "SDK release failed"
        code, output, _ = self.invoke("--execute")
        self.assertEqual(code, 1)
        self.assertIn("SDK release failed", output)

    def test_reset_is_explicit_and_precedes_motion(self):
        self.driver.sample.payload.arms["left"].error = 13
        code, _, _ = self.invoke("--execute", "--reset")
        self.assertEqual(code, 0)
        self.assertEqual(self.driver.calls[:3], ["start", "reset", "configure"])

    def test_invalid_target_rejected_before_connection(self):
        self.source["target_deg"]["right"] = [0] * 6
        code, output, factory = self.invoke("--execute")
        self.assertEqual(code, 1)
        factory.assert_not_called()
        self.assertIn("seven finite", output)


if __name__ == "__main__":
    unittest.main()
