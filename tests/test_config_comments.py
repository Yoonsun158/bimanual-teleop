"""YAML comments, parse errors, and shared device configuration."""

from pathlib import Path
import tempfile
import unittest

import yaml

from bimanual_teleop.common.config import load_yaml_config
from bimanual_teleop.control.hand.follow import create_wuji_teleop
from bimanual_teleop.devices.wuji.config import load_config as load_wuji_config
from bimanual_teleop.devices.tianji.config import load_config as load_tianji_config
from bimanual_teleop.devices.tianji.model import MotionProfile
from bimanual_teleop.devices.wuji.config import DEFAULT_CONFIG as WUJI_CONFIG, glove_settings
from bimanual_teleop.types import ControlProfile


class CommentedConfigTests(unittest.TestCase):
    def setUp(self):
        temporary = tempfile.TemporaryDirectory()
        self.addCleanup(temporary.cleanup)
        self.path = Path(temporary.name) / "config.yaml"

    def test_strings_escaping_unicode_and_comments(self):
        values = {"url": "https://example.com/a//b", "text": '中文 "引号" /*保留*/ //保留',
                  "path": "C:\\tools\\", "literal": "\\n", "numbers": [1, -2.5, 3]}
        self.path.write_text("# 文件说明\n" + yaml.safe_dump(values, allow_unicode=True), encoding="utf-8")
        self.assertEqual(load_yaml_config(self.path), values)
        self.path.write_text('a: 1 # 第一项\nb: "保留 # 字符"\n', encoding="utf-8")
        self.assertEqual(load_yaml_config(self.path), {"a": 1, "b": "保留 # 字符"})

    def test_invalid_yaml_reports_file_and_error_location(self):
        self.path.write_text('# 说明\na: [1, 2\n', encoding="utf-8")
        with self.assertRaises(ValueError) as caught:
            load_yaml_config(self.path)
        self.assertIn(str(self.path), str(caught.exception))
        self.assertIn("line 3", str(caught.exception))

    def test_nonmapping_documents_and_python_tags_are_rejected(self):
        for text in ('', '# 只有注释', '- item', 'scalar', '!!python/tuple [1, 2]'):
            self.path.write_text(text, encoding="utf-8")
            with self.subTest(text=text), self.assertRaises(ValueError):
                load_yaml_config(self.path)

    def test_wuji_viewer_calibration_teleop_and_home_share_commented_config(self):
        raw = load_yaml_config(WUJI_CONFIG)
        self.assertNotIn("profile_id", raw)
        self.assertNotIn("mode", raw)
        self.assertNotIn("parameter_source", raw)
        for side in ("left", "right"):
            address, user = glove_settings(WUJI_CONFIG, side, user_name="yuchen")
            self.assertEqual(address, raw["devices"][side]["glove"])
            self.assertEqual(user, {"user_name": "yuchen"})
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
