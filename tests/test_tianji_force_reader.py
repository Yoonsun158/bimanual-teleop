"""Continuous force output and cleanup with a simulated Tianji SDK."""

from contextlib import redirect_stderr, redirect_stdout
import io
from pathlib import Path
import tempfile
from types import SimpleNamespace
import unittest
from unittest.mock import Mock, patch

from scripts import read_tianji_right_force as force


def frame(sequence):
    return SimpleNamespace(m_Out=[None, SimpleNamespace(m_OutFrameSerial=sequence,
        m_EST_Joint_Firc=[216], m_EST_Joint_Firc_Dot=[10000, -20000, 30000, 4000, -5000, 6000])])


class ForceReaderTests(unittest.TestCase):
    def run_reader(self, values, *, clock=None):
        robot = Mock()
        robot.read.side_effect = values
        sdk = SimpleNamespace(DCSS=Mock(), Marvin_Robot=Mock(return_value=robot))
        with tempfile.TemporaryDirectory() as directory, \
                patch.object(force, "ControlSDK", return_value=robot), \
                patch.object(force.sys, "path", list(force.sys.path)), \
                patch.object(force.time, "sleep"), \
                patch.object(force.time, "monotonic", side_effect=clock, return_value=0.), \
                redirect_stdout(io.StringIO()) as output, redirect_stderr(io.StringIO()) as error:
            root = Path(directory)
            (root / "SDK_PYTHON").mkdir()
            (root / "SDK_PYTHON/fx_robot.py").touch()
            code = force.main(["--sdk-root", str(root)])
        return code, output.getvalue(), error.getvalue(), robot

    def test_continues_past_ten_frames_until_interrupted_and_releases_connection(self):
        code, output, error, robot = self.run_reader([*(frame(i) for i in range(1, 13)), KeyboardInterrupt()])
        self.assertEqual(code, 0)
        self.assertEqual(output.count("frame="), 12)
        self.assertIn("F[N]=(1.0000, -2.0000, 3.0000)", output)
        self.assertIn("T[N·m]=(0.4000, -0.5000, 0.6000)", output)
        self.assertIn("Ctrl+C", error)
        robot.close.assert_called_once()

    def test_stale_data_timeout_still_releases_connection(self):
        code, _, error, robot = self.run_reader([frame(0)], clock=[0., 3.1])
        self.assertEqual(code, 1)
        self.assertIn("no fresh right-arm force data", error)
        robot.close.assert_called_once()


if __name__ == "__main__":
    unittest.main()
