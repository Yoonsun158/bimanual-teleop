"""Recording CPU partitions are topology-aware and best effort."""

import unittest
from unittest.mock import patch

from bimanual_teleop.common.affinity import apply_recording_affinity, recording_cpu_sets


class RecordingAffinityTests(unittest.TestCase):
    def test_four_physical_cores_split_control_from_background(self):
        groups = [(0, 4), (1, 5), (2, 6), (3, 7)]
        with patch("bimanual_teleop.common.affinity.os.sched_getaffinity",
                   return_value=set(range(8))), patch(
                       "bimanual_teleop.common.affinity._physical_groups",
                       return_value=groups):
            self.assertEqual(recording_cpu_sets(), {
                "control": frozenset((0, 4, 1, 5)),
                "background": frozenset((2, 6, 3, 7)),
            })

    def test_unknown_or_small_topology_disables_partition(self):
        with patch("bimanual_teleop.common.affinity.os.sched_getaffinity",
                   return_value={0, 1}), patch(
                       "bimanual_teleop.common.affinity._physical_groups",
                       return_value=[(0,), (1,)]):
            self.assertIsNone(recording_cpu_sets())

    def test_apply_is_best_effort(self):
        with patch("bimanual_teleop.common.affinity.recording_cpu_sets",
                   return_value={"control": frozenset((0, 4)),
                                 "background": frozenset((1, 5))}), patch(
                       "bimanual_teleop.common.affinity.os.sched_setaffinity") as setter:
            self.assertEqual(apply_recording_affinity("background"), (1, 5))
            setter.assert_called_once_with(0, frozenset((1, 5)))


if __name__ == "__main__":
    unittest.main()
