"""All Tianji entry points share device, motion and Quest configuration."""

from pathlib import Path
import tempfile
import unittest

import yaml

from bimanual_teleop.devices.tianji.config import DEFAULT_CONFIG, load_config
from bimanual_teleop.common.config import load_yaml_config


class TianjiConfigTests(unittest.TestCase):
    def test_default_and_override(self):
        settings = load_config()
        self.assertEqual(settings["controller_ip"], "192.168.1.190")
        source = load_yaml_config(DEFAULT_CONFIG)
        self.assertNotIn("profile_id", source["profile"])
        self.assertNotIn("mode", source["profile"])
        self.assertEqual(settings["profile"]["profile_id"], "tianji-teleop")
        self.assertEqual(settings["profile"]["mode"], "cartesian_impedance")
        source["profile"].update(profile_id="tianji-teleop", mode="cartesian_impedance")
        self.assertEqual(source, settings)
        self.assertEqual(settings["quest"]["coordinate_frame"], "world")
        self.assertEqual(settings["profile"]["parameters"]["active_arms"], ["left", "right"])
        self.assertEqual(settings["ready_pose"]["velocity_ratio"], 25)
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "config.yaml"
            path.write_text('controller_ip: 192.0.2.8\n')
            self.assertEqual(load_config(path)["controller_ip"], "192.0.2.8")
            self.assertEqual(load_config(path, "192.0.2.9")["controller_ip"], "192.0.2.9")

    def test_invalid_address_fails_before_device_connection(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "config.yaml"
            for source in ('{}', 'controller_ip: robot.local\n',
                           'controller_ip: 999.1.1.1\n'):
                with self.subTest(source=source):
                    path.write_text(source)
                    with self.assertRaises(ValueError):
                        load_config(path)

    def test_coordinate_frame_defaults_and_supported_choices(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "config.yaml"
            for quest in ({}, {"coordinate_frame": "headset"}, {"coordinate_frame": "world"}):
                path.write_text(yaml.safe_dump({"controller_ip": "192.0.2.8", "quest": quest}))
                self.assertEqual(load_config(path)["quest"]["coordinate_frame"],
                                 quest.get("coordinate_frame", "headset"))
            path.write_text('controller_ip: 192.0.2.8\n')
            self.assertEqual(load_config(path)["quest"]["coordinate_frame"], "headset")
            for value in ("local", "", None):
                path.write_text(yaml.safe_dump({"controller_ip": "192.0.2.8",
                                            "quest": {"coordinate_frame": value}}))
                with self.assertRaisesRegex(ValueError, "coordinate_frame"):
                    load_config(path)


if __name__ == "__main__":
    unittest.main()
