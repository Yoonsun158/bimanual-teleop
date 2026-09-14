"""Comments must not alter values, mask JSON errors, or break device entry points."""

import json
from pathlib import Path
import tempfile
import unittest

from bimanual_teleop.common.config import load_json_config
from bimanual_teleop.control.hand.follow import create_wuji_teleop, load_config as load_wuji_config
from bimanual_teleop.devices.tianji.config import load_config as load_tianji_config
from bimanual_teleop.devices.tianji.model import MotionProfile
from bimanual_teleop.devices.wuji.config import DEFAULT_CONFIG as WUJI_CONFIG, glove_settings
from bimanual_teleop.types import ControlProfile


class CommentedConfigTests(unittest.TestCase):
    def setUp(self):
        temporary = tempfile.TemporaryDirectory()
        self.addCleanup(temporary.cleanup)
        self.path = Path(temporary.name) / "config.json"

    def test_strings_escaping_unicode_and_both_comment_styles(self):
        values = {"url": "https://example.com/a//b", "text": '中文 "引号" /*保留*/ //保留',
                  "path": "C:\\tools\\", "literal": "\\n", "numbers": [1, -2.5, 3]}
        self.path.write_text("// 文件说明\n/* 多行\n中文注释 */\n" + json.dumps(values, ensure_ascii=False)
                             + " // 行末注释\n", encoding="utf-8")
        self.assertEqual(load_json_config(self.path), values)
        self.path.write_text('{"a": /* 值说明 */ 1, // 第一项\n "b": 2}', encoding="utf-8")
        self.assertEqual(load_json_config(self.path), {"a": 1, "b": 2})

    def test_invalid_json_is_rejected_with_original_error_location(self):
        self.path.write_text('// 说明\n{\n  "a": 1, // 不能用注释隐藏尾逗号\n}\n', encoding="utf-8")
        with self.assertRaises(json.JSONDecodeError) as caught:
            load_json_config(self.path)
        self.assertEqual((caught.exception.lineno, caught.exception.colno), (4, 1))
        for text in ('{"a": 1 /* 未结束}', '{"a": "未结束 // 字符串}', '{"a": tru/*x*/e}'):
            self.path.write_text(text, encoding="utf-8")
            with self.assertRaises(json.JSONDecodeError):
                load_json_config(self.path)

    def test_wuji_viewer_calibration_teleop_and_home_share_commented_config(self):
        raw = load_json_config(WUJI_CONFIG)
        self.assertNotIn("profile_id", raw)
        self.assertNotIn("mode", raw)
        self.assertNotIn("parameter_source", raw)
        for side in ("left", "right"):
            address, user = glove_settings(WUJI_CONFIG, side, user_name="yuchen")
            self.assertEqual(address, raw["devices"][side]["glove"])
            self.assertEqual(user, {"user_name": "yuchen", "user_id": ""})
        loaded = load_wuji_config(WUJI_CONFIG)
        # Home uses the loaded config; direct callers may pass the unadorned dict.
        home_profile = ControlProfile(loaded["profile_id"], loaded.get("mode", "mit"), loaded["parameters"])
        runtime = create_wuji_teleop(raw)
        self.assertEqual(runtime.profile, home_profile)
        self.assertEqual(runtime.profile.profile_id, "wuji-hand2")
        self.assertEqual(runtime.profile.parameters, raw["parameters"])
        self.assertIsNone(runtime.session.manager)

    def test_tianji_profile_without_bookkeeping_fields_is_usable(self):
        settings = load_tianji_config()
        parsed = MotionProfile.from_control_profile(ControlProfile(**settings["profile"]))
        self.assertEqual(parsed.active_arms, ("left", "right"))
        self.assertEqual(parsed.arms["left"].stiffness, (900., 900., 900., 90., 90., 90.))
        self.assertEqual(set(settings["ready_pose"]),
                         {"order", "target_deg", "velocity_ratio", "acceleration_ratio"})


if __name__ == "__main__":
    unittest.main()
