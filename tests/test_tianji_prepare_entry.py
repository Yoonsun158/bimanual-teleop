"""Tianji preparation and home entries share the configured ready-pose command."""

from pathlib import Path
import sys
import unittest
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "scripts"))
import prepare_tianji_teleop as cli
import home_tianji as home

from bimanual_teleop.devices.tianji.config import DEFAULT_CONFIG


class PreparationEntryTests(unittest.TestCase):
    def test_default_selects_motion_after_confirmation(self):
        with patch.object(cli, "run", return_value=0) as run:
            self.assertEqual(cli.main([]), 0)
        self.assertFalse(run.call_args.args[0].inspect)

    def test_execute_arguments_and_failure_are_forwarded(self):
        with patch.object(cli, "run", return_value=1) as run:
            self.assertEqual(cli.main(["--ip", "192.0.2.1"]), 1)
        args = run.call_args.args[0]
        self.assertFalse(args.inspect)
        self.assertEqual(args.ip, "192.0.2.1")

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
                "--ip", "192.0.2.9", "--library", "custom.so", "--model", "model.MvKDCfg"]), 1)
        args = run.call_args.args[0]
        self.assertFalse(args.inspect)
        self.assertEqual(args.side, "left")
        self.assertEqual(args.config, Path("custom.yaml"))
        self.assertEqual(args.ip, "192.0.2.9")
        self.assertEqual(args.library, Path("custom.so"))
        self.assertEqual(args.model, Path("model.MvKDCfg"))
