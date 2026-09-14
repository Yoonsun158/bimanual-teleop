"""Named SDK users and tactile layout checks without physical hardware."""

from contextlib import redirect_stderr
from io import StringIO
from types import SimpleNamespace as NS
import unittest
from unittest.mock import Mock, patch

from bimanual_teleop.cli.calibrate_wuji_glove import CalibrationGuide
from bimanual_teleop.calibration.wuji import (
    calibrate_glove, require_tactile_744, resolve_user_id,
)
from bimanual_teleop.devices.wuji.adapter import WujiSdkSession, WujiTactileFrame
from bimanual_teleop.types import Sample, SampleHeader, SampleRef


def sample(columns, valid=True):
    return Sample(SampleHeader(SampleRef("tactile", "epoch", 0), 1, valid),
                  WujiTactileFrame(24, columns, (0.,) * (24 * columns)))


class CalibrationTests(unittest.TestCase):
    def test_tactile_calibration_accepts_744_but_never_truncates_768(self):
        source = NS(get_latest_stream=lambda stream: sample(31))
        self.assertEqual(require_tactile_744(source).payload.columns, 31)
        source.get_latest_stream = lambda stream: sample(32)
        with self.assertRaisesRegex(RuntimeError, "744.*24×32"):
            require_tactile_744(source)
        source.get_latest_stream = lambda stream: sample(31, valid=False)
        with self.assertRaisesRegex(RuntimeError, "invalid"):
            require_tactile_744(source)

    def test_tactile_result_without_installed_model_is_not_success(self):
        user = {"user_id": "named", "display_name": "Alice", "is_default": False}
        manager = NS(list_users=lambda: [user])
        session, source = Mock(), Mock()
        source._device.calibrate_tactile_blocking.return_value = {"installed": False}
        with patch("bimanual_teleop.calibration.wuji.WujiSdkSession", return_value=session), \
             patch("bimanual_teleop.calibration.wuji.WujiGloveSource", return_value=source), \
             patch("bimanual_teleop.calibration.wuji.require_tactile_744"):
            with self.assertRaisesRegex(RuntimeError, "未安装触觉模型"):
                calibrate_glove("left", "test", kind="tactile", user_id="named",
                                manager=manager, sdk=NS())
        source.close.assert_called_once()
        session.close.assert_called_once()

    def test_named_user_exact_match_or_create(self):
        manager = NS(list_users=Mock(return_value=[
            {"user_id": "a", "display_name": "Alice", "is_default": False}]),
            create_user=Mock(return_value={"user_id": "b"}))
        self.assertEqual(resolve_user_id(manager, user_name="Alice", create=True), "a")
        manager.create_user.assert_not_called()
        self.assertEqual(resolve_user_id(manager, user_name="Bob", create=True), "b")
        manager.create_user.assert_called_once_with("Bob")
        with self.assertRaisesRegex(ValueError, "首次标定请用 --user-name"):
            resolve_user_id(manager, user_id="missing")

    def test_joint_guide_names_each_hand_pose_and_explains_auto_capture(self):
        output = StringIO()
        with redirect_stderr(output):
            guide = CalibrationGuide("joints")
            guide.feedback({"step_index": 0, "step_total": 6, "step_name": "pinch_index",
                            "state": "waiting_stable", "constraints_ok": False,
                            "hints": ["请轻触指尖"]})
            guide.feedback({"step_index": 0, "step_total": 6, "step_name": "pinch_index",
                            "state": "done"})
            guide.feedback({"step_index": 1, "step_total": 6, "step_name": "pinch_middle",
                            "state": "collecting"})
        shown = output.getvalue()
        self.assertIn("[1/6] 拇指和食指仅用指尖轻触", shown)
        self.assertIn("保持手腕和手指不动", shown)
        self.assertIn("现在完全张开手", shown)
        self.assertIn("[2/6] 拇指和中指仅用指尖轻触", shown)
        self.assertIn("请轻触指尖", shown)

    def test_joint_guide_attributes_opposite_angle_hints_to_each_finger(self):
        feedback = {
            "step_index": 4, "step_total": 6, "step_name": "four_finger_bend_90",
            "state": "waiting_stable", "constraints_ok": False,
            "metrics": [
                {"finger": "index", "label": "roll", "value": -18,
                 "min": -10, "max": 10, "unit": "deg",
                 "hint": "rotate to increase roll"},
                {"finger": "pinky", "label": "roll", "value": 18,
                 "min": -10, "max": 10, "unit": "deg",
                 "hint": "rotate to decrease roll"},
            ],
            "hints": ["rotate to increase roll", "rotate to decrease roll"],
        }
        output = StringIO()
        with redirect_stderr(output):
            CalibrationGuide("joints").feedback(feedback)
        shown = output.getvalue()
        self.assertIn("食指 roll；当前 -18°；目标 -10～10°；使 roll 增大", shown)
        self.assertIn("小指 roll；当前 18°；目标 -10～10°；使 roll 减小", shown)
        self.assertNotIn("SDK 提示：", shown)

    def test_joint_guide_flags_persistent_scale_yaw_mismatch(self):
        output = StringIO()
        with redirect_stderr(output):
            CalibrationGuide("joints").feedback({
                "step_index": 4, "step_total": 6, "step_name": "four_finger_bend_90",
                "state": "waiting_stable", "constraints_ok": False,
                "metrics": [
                    {"finger": "index", "label": "Yaw", "value": -174,
                     "min": -40, "max": 35, "unit": "deg"},
                    {"finger": "middle", "label": "Yaw", "value": 174,
                     "min": -40, "max": 35, "unit": "deg"},
                ],
            })
        shown = output.getvalue()
        self.assertIn("多指 Yaw 接近 ±180°", shown)
        self.assertIn("手套固件", shown)
        self.assertNotIn("逐根调整弯曲", shown)
        self.assertNotIn("使 yaw", shown)

    def test_tactile_guide_separates_start_and_review_choices(self):
        ready = {"kind": "pose_ready", "step_index": 0, "step_total": 4,
                 "step_name": "four_finger_L_thumb_in", "seconds_per_pose": 30}
        review = {"kind": "pose_review", "step_index": 0, "step_total": 4,
                  "step_name": "four_finger_L_thumb_in", "warnings": ["采集帧数不足"]}
        output = StringIO()
        with redirect_stderr(output), patch("builtins.input", side_effect=["r", "", "r"]):
            guide = CalibrationGuide("tactile")
            self.assertEqual(guide.pose_prompt(ready), "proceed")
            self.assertEqual(guide.pose_prompt(review), "retry")
        shown = output.getvalue()
        self.assertIn("[1/4] 四指在根部关节向下弯约 90°", shown)
        self.assertIn("回车后持续做本步动作约 30 秒", shown)
        self.assertIn("采集质量：采集帧数不足", shown)
        self.assertIn("请输入回车或提示中的选项", shown)
        with patch("builtins.input", side_effect=EOFError), redirect_stderr(StringIO()):
            self.assertEqual(guide.pose_prompt(ready), "abort")

    def test_calibration_failure_still_restores_user_and_closes_glove(self):
        user = {"user_id": "named", "display_name": "Alice", "is_default": False}
        manager = NS(list_users=lambda: [user], current_user=Mock(return_value=user),
                     switch_user=Mock(return_value=user), disconnect=Mock())
        glove = Mock()
        glove.hand_side.return_value.get.return_value = "left"
        glove.info = NS(firmware_version="1")
        glove.serial_number = "serial"
        glove.is_connected = True
        glove.emf_poses.return_value.subscribe.return_value.recv.return_value = None
        glove.hand_skeleton.return_value.subscribe.return_value.recv.return_value = None
        glove.hand_joint_angles.return_value.subscribe.return_value.recv.return_value = None
        glove.tactile.return_value.subscribe.return_value.recv.return_value = None
        glove.hand_model_path.return_value.get.return_value = "model.urdf"
        glove.calibrate_blocking.side_effect = RuntimeError("calibration failed")
        manager.connect = Mock(return_value=glove)
        sdk = NS(WujiGlove=type(glove), ConnectOptions=lambda **kwargs: NS(**kwargs))
        with self.assertRaisesRegex(RuntimeError, "calibration failed"):
            calibrate_glove("left", "test", kind="joints", user_id="named",
                            sdk=sdk, manager=manager)
        manager.disconnect.assert_called_once()
        manager.switch_user.assert_called()

    def test_calibration_user_name_can_be_reselected_for_acquisition(self):
        class Manager:
            user = "prior"

            def list_users(self):
                return [{"user_id": "named", "display_name": "Alice", "is_default": False}]

            def current_user(self):
                return {"user_id": self.user,
                        "display_name": "Alice" if self.user == "named" else self.user,
                        "is_default": self.user == ""}

            def switch_user(self, user_id):
                self.user = user_id
                return self.current_user()

        manager = Manager()
        device = Mock()
        device.hand_side.return_value.get.return_value = "left"
        device.info = NS(firmware_version="1")
        device.serial_number = "serial"
        device.is_connected = True
        for method in (device.emf_poses, device.hand_skeleton,
                       device.hand_joint_angles, device.tactile):
            method.return_value.subscribe.return_value.recv.return_value = None
        device.hand_model_path.return_value.get.return_value = ""
        device.calibrate_blocking.return_value = {"calibrated_urdf": "Alice.urdf"}
        manager.connect = Mock(return_value=device)
        manager.disconnect = Mock()
        sdk = NS(WujiGlove=type(device), ConnectOptions=lambda **kwargs: NS(**kwargs))
        result = calibrate_glove("left", "test", kind="joints", user_name="Alice",
                                 manager=manager, sdk=sdk)
        self.assertEqual(result["sdk_user"]["user_id"], "named")
        self.assertEqual(result["model"], "Alice.urdf")
        self.assertEqual(manager.user, "prior")
        session = WujiSdkSession(user_name=result["sdk_user"]["display_name"],
                                 manager=manager, sdk=sdk).open()
        self.assertEqual(session.metadata["sdk_user_name"], "Alice")
        self.assertEqual(session.metadata["sdk_user_id"], "named")
        self.assertTrue(session.health().ready)
        session.close()
        self.assertEqual(manager.user, "prior")


if __name__ == "__main__":
    unittest.main()
