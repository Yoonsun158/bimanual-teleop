"""Compile the actual native collector with a small XR test double."""

import json
from pathlib import Path
import shutil
import subprocess
import sys
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from bimanual_teleop.devices.quest.adapter import decode_message  # noqa: E402


class QuestNativeTest(unittest.TestCase):
    def test_native_queries_and_wire_protocol(self):
        compiler = shutil.which("c++")
        if compiler is None:
            self.skipTest("C++ compiler is not available")
        root = Path(__file__).resolve().parents[1]
        with tempfile.TemporaryDirectory() as directory:
            executable = Path(directory) / "quest-native-test"
            subprocess.run(
                [
                    compiler, "-std=c++20", "-I", str(root / "tests/native"),
                    str(root / "quest_app/src/main/cpp/quest_capture.cpp"),
                    "-o", str(executable),
                ],
                check=True, capture_output=True, text=True,
            )
            output = subprocess.run(
                [str(executable)], check=True, capture_output=True, text=True,
            ).stdout
        records = [json.loads(line) for line in output.splitlines()]
        # Exercise the real Python decoder on C++ output, including events.
        decoded = [decode_message(line, 1) for line in output.splitlines()]
        self.assertEqual(len(decoded), len(records))
        frames = [row for row in records if row["type"] == "frame"]
        self.assertEqual([row["seq"] for row in frames], [0, 1, 2])
        self.assertEqual([row["xr_time"] for row in frames], [1000, 1200, 1300])
        self.assertEqual([row["origin"] for row in frames], [0, 1, 1])
        self.assertEqual([row["refresh_hz"] for row in frames], [90, 90, 72])
        for row in frames:
            self.assertEqual(row["session"], "0123456789abcdef0123456789abcdef")
            self.assertLessEqual(row["query_ns"], row["send_ns"])
            self.assertEqual(row["state"], 5)
            self.assertEqual(row["head"]["p"], [1, 2, 3])
            self.assertIsNone(row["head"]["active"])
            self.assertIsNone(row["left"]["p"])
            self.assertEqual(row["left"]["q"], [0, 0, 0, 1])
            self.assertTrue(row["left"]["active"])
            self.assertEqual(row["right"]["p"], [4, 2, 3])
            self.assertIsNone(row["right"]["q"])
            self.assertFalse(row["right"]["active"])
            self.assertLess(len(json.dumps(row)), 2048)
        change = next(row for row in records if row.get("event") == "reference_space_change")
        self.assertLess(records.index(change), records.index(frames[0]))
        self.assertEqual(change["details"]["change_time"], 1200)
        self.assertEqual(change["details"]["pose_in_previous_space"], {"p": None, "q": None})
        self.assertEqual(records[-1]["event"], "error")
        self.assertEqual(records[-1]["details"], {"operation": "xrRequestDisplayRefreshRateFB", "result": -1})


if __name__ == "__main__":
    unittest.main()
