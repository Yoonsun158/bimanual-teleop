"""Tianji home entry forwards configuration and device overrides."""

from pathlib import Path
import unittest
from unittest.mock import patch

from bimanual_teleop.cli import home_tianji as home

from bimanual_teleop.devices.tianji.config import DEFAULT_CONFIG


class HomeEntryTests(unittest.TestCase):
    def test_home_defaults_to_both_arms(self):
        with patch.object(home, "run", return_value=0) as run:
            self.assertEqual(home.main([]), 0)
        args = run.call_args.args[0]
        self.assertFalse(args.inspect)
        self.assertEqual(args.side, "both")
        self.assertEqual(args.config, DEFAULT_CONFIG)

    def test_home_preserves_config_and_driver_overrides(self):
        with patch.object(home, "run", return_value=1) as run:
            self.assertEqual(home.main([
                "--side", "left", "--tianji-config", "custom.yaml",
                "--ip", "192.0.2.9", "--sdk-root", "custom.so", "--model", "model.MvKDCfg"]), 1)
        args = run.call_args.args[0]
        self.assertFalse(args.inspect)
        self.assertEqual(args.side, "left")
        self.assertEqual(args.config, Path("custom.yaml"))
        self.assertEqual(args.ip, "192.0.2.9")
        self.assertEqual(args.sdk_root, Path("custom.so"))
        self.assertEqual(args.model, Path("model.MvKDCfg"))
