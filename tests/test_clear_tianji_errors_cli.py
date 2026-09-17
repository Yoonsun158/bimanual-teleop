"""Standalone clearing never configures or enables a controller."""

from contextlib import redirect_stderr
import io
import unittest
from unittest.mock import patch

from bimanual_teleop.cli import clear_tianji_errors as cli


class ClearEntryTests(unittest.TestCase):
    def test_selected_arm_and_connection_options_reach_one_driver(self):
        with patch.object(cli, "TianjiDriver") as factory, redirect_stderr(io.StringIO()):
            factory.return_value.clear_errors.return_value = {"requested_arms": ["right"]}
            self.assertEqual(cli.main(["--side", "right", "--ip", "192.0.2.9"]), 0)
        factory.assert_called_once_with("192.0.2.9", None)
        driver = factory.return_value
        driver.clear_errors.assert_called_once_with(("right",))
        driver.close.assert_called_once()
        driver.configure.assert_not_called()
        driver.engage.assert_not_called()
        driver.move_joints.assert_not_called()

    def test_persistent_error_fails_and_still_releases_connection(self):
        with patch.object(cli, "TianjiDriver") as factory, redirect_stderr(io.StringIO()) as output:
            factory.return_value.clear_errors.side_effect = RuntimeError("left controller 13")
            self.assertEqual(cli.main([]), 1)
        self.assertIn("left controller 13", output.getvalue())
        factory.return_value.close.assert_called_once()


if __name__ == "__main__":
    unittest.main()
