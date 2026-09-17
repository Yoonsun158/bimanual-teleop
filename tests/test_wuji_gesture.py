"""Gesture geometry and sample-driven commands without SDK/device access."""

from dataclasses import replace
import math
import unittest
from unittest.mock import patch

from bimanual_teleop.devices.wuji.adapter import SKELETON_NAMES
from bimanual_teleop.control.hand.gesture import GestureCommands, classify_gestures


from tests.support.gesture import skeleton, ROCK, V, OPEN, frame, recorded_pose


class GeometryTests(unittest.TestCase):
    def test_open_palms_require_all_fingers_extended(self):
        self.assertIs(classify_gestures(OPEN)[2], True)
        for pose in (ROCK, V, skeleton((140,)*5), skeleton((0,)*5, mcp=(0, 90, 0, 0, 0))):
            self.assertIsNot(classify_gestures(pose)[2], True)
        self.assertIsNone(classify_gestures(replace(OPEN, confidences=(0.,)*21))[2])
    def test_recorded_v_and_rock_samples(self):
        for name in ("v_left_early", "v_right_early", "v_left_later", "v_right_later"):
            with self.subTest(name=name):
                self.assertIs(classify_gestures(recorded_pose(name))[1], True)
                self.assertIsNot(classify_gestures(recorded_pose(name))[0], True)
        for name in ("rock_left_recorded", "rock_right_recorded"):
            self.assertIs(classify_gestures(recorded_pose(name))[0], True)
            self.assertIs(classify_gestures(recorded_pose(name))[1], False)
        # The operator reported V throughout; these frames do not establish it.
        for name in ("low_extension_left", "low_extension_right"):
            self.assertIsNot(classify_gestures(recorded_pose(name))[1], True)

    def test_v_accounts_for_mcp_flexion_even_when_distal_joints_are_straight(self):
        self.assertIs(classify_gestures(skeleton((90, 0, 0, 0, 0), mcp=(0, 0, 0, 90, 90)))[1], True)
        self.assertIs(classify_gestures(skeleton((90, 0, 0, 140, 140), mcp=(0, 90, 90, 0, 0)))[1], False)

    def test_rock_wins_when_distal_angles_and_projection_disagree(self):
        pose = skeleton((90, 0, 80, 140, 0), mcp=(0, 0, 0, 0, 90))
        points = list(pose.positions_m)
        # An S-shaped middle finger passes extension while legacy bend sees 80°.
        point = points[9]
        for joint, angle in zip((10, 11, 12), (0, 40, 0)):
            angle = math.radians(angle)
            point = (point[0], point[1] + .02 * math.cos(angle), point[2] + .02 * math.sin(angle))
            points[joint] = point
        pose = replace(pose, positions_m=tuple(points))
        self.assertIs(classify_gestures(pose)[0], True)
        self.assertIs(classify_gestures(pose)[1], False)
        samples = {s: None for s in ("left", "right")}
        commands = GestureCommands({s: lambda s=s: samples[s] for s in samples})
        actions = []
        for sequence in range(12):
            stamp = 1_000_000_000 + sequence * 100_000_000
            samples = {s: frame(s, pose, stamp, sequence) for s in samples}
            action = commands.poll(stamp)
            if action:
                actions.append(action)
        self.assertEqual(actions, [("pause", ("left", "right"))])

    def test_horns_require_thumb_middle_and_ring_bent(self):
        self.assertIs(classify_gestures(ROCK)[0], True)
        for bends in ((0,) * 5, (140,) * 5, (0, 0, 140, 140, 0),
                      (90, 0, 0, 140, 0), (90, 0, 140, 0, 0), (90, 90, 140, 140, 0)):
            with self.subTest(bends=bends):
                self.assertIs(classify_gestures(skeleton(bends))[0], False)

    def test_v_requires_only_index_and_middle_extended(self):
        self.assertIs(classify_gestures(V)[1], True)
        for bends in ((0,) * 5, (140,) * 5, (0, 0, 0, 140, 140),
                      (90, 90, 0, 140, 140), (90, 0, 140, 140, 140),
                      (90, 0, 0, 0, 140), (90, 0, 0, 140, 0)):
            with self.subTest(bends=bends):
                self.assertIsNot(classify_gestures(skeleton(bends))[1], True)
        self.assertIs(classify_gestures(ROCK)[1], False)
        self.assertIs(classify_gestures(V)[0], False)

    def test_rotation_translation_mirroring_and_scale_preserve_both_gestures(self):
        for pose, index in ((ROCK, 0), (V, 1)):
            for scale in (.5, 1., 2.):
                for mirror in (-1, 1):
                    transformed = tuple((.2 + scale * z, -.3 + scale * mirror * x, .1 - scale * y)
                                        for x, y, z in pose.positions_m)
                    self.assertIs(classify_gestures(replace(pose, positions_m=transformed))[index], True)

    def test_uncertain_bend_and_invalid_geometry_are_not_release(self):
        for pose, index in ((ROCK, 0), (V, 1)):
            uncertain = [replace(pose, confidences=(.1,) * 21),
                         replace(pose, confidences=(float("nan"),) * 21),
                         replace(pose, positions_m=((0., 0., 0.),) * 21),
                         replace(pose, positions_m=((float("inf"), 0., 0.),) * 21),
                         replace(pose, positions_m=pose.positions_m[:-1]),
                         replace(pose, joint_names=tuple(reversed(SKELETON_NAMES)))]
            for invalid in uncertain:
                self.assertIsNone(classify_gestures(invalid)[index])
        self.assertIsNone(classify_gestures(skeleton((90, 50, 140, 140, 0)))[0])
        self.assertIsNone(classify_gestures(skeleton((90, 75, 0, 140, 140)))[1])


class CommandTests(unittest.TestCase):
    def setUp(self):
        self.samples = {"left": None, "right": None}
        self.now, self.sequence = 1_000_000_000, 0
        self.commands = GestureCommands({s: lambda s=s: self.samples[s] for s in self.samples})

    def feed(self, left=OPEN, right=OPEN, *, dt=.1, epoch="test", start_ready=True):
        self.now += round(dt * 1e9)
        self.sequence += 1
        self.samples = {s: None if pose is None else frame(s, pose, self.now, self.sequence, epoch=epoch)
                        for s, pose in (("left", left), ("right", right))}
        return self.commands.poll(self.now, start_ready=start_ready)

    def hold(self, left=OPEN, right=OPEN, *, count=4):
        return [self.feed(left, right) for _ in range(count)]

    def test_both_v_start_once_until_a_new_pose(self):
        self.assertEqual(self.hold(V, V), [None, None, None, ("engage", ("left", "right"))])
        self.assertEqual(self.hold(V, V, count=20), [None] * 20)
        self.feed(OPEN, V)
        self.assertEqual(self.hold(V, V), [None, None, None, ("engage", ("left", "right"))])

    def test_one_v_or_missing_other_hand_cannot_start(self):
        for left, right in ((V, OPEN), (OPEN, V), (V, None), (None, V)):
            self.assertEqual(self.hold(left, right, count=10), [None] * 10)

    def test_either_rock_stops_once_and_both_are_one_event(self):
        for left, right, sides in ((ROCK, V, ("left",)), (V, ROCK, ("right",)),
                                  (ROCK, ROCK, ("left", "right"))):
            self.setUp()
            self.assertEqual(self.hold(left, right), [None, None, None, ("pause", sides)])
            self.assertEqual(self.hold(left, right, count=20), [None] * 20)

    def test_new_rock_can_stop_without_both_hands_releasing(self):
        self.hold(ROCK, OPEN)
        self.assertEqual(self.hold(ROCK, ROCK), [None, None, None, ("pause", ("right",))])
        self.feed(V, ROCK)
        self.assertEqual(self.hold(ROCK, ROCK), [None, None, None, ("pause", ("left",))])

    def test_unavailable_or_uncertain_other_hand_cannot_block_stop(self):
        for pose in (None, skeleton((90, 50, 140, 140, 0)), replace(OPEN, confidences=(0.,) * 21)):
            self.setUp()
            self.assertEqual(self.hold(pose, ROCK), [None, None, None, ("pause", ("right",))])

    def test_short_gesture_does_not_trigger(self):
        for left, right in ((V, V), (ROCK, OPEN)):
            self.setUp()
            self.assertEqual(self.hold(left, right, count=3), [None] * 3)
            self.assertIsNone(self.feed())
            self.assertEqual(self.hold(left, right)[:3], [None] * 3)

    def test_v_confirmation_intervals_must_overlap_for_full_hold(self):
        self.feed(V, OPEN)
        self.hold(V, V, count=3)
        self.now += 100_000_000
        self.sequence += 1
        self.samples["right"] = frame("right", V, self.now, self.sequence)
        # Both hands individually have 300 ms of V, but overlap is only 200 ms.
        self.assertIsNone(self.commands.poll(self.now))
        self.samples["left"] = frame("left", V, self.now, self.sequence)
        self.assertEqual(self.commands.poll(self.now), ("engage", ("left", "right")))

    def test_cached_or_repeated_source_frames_cannot_complete_dwell(self):
        self.feed(ROCK, OPEN)
        for _ in range(8):
            self.now += 100_000_000
            self.assertIsNone(self.commands.poll(self.now))
        for _ in range(8):
            self.now += 100_000_000
            self.samples["left"] = frame("left", ROCK, self.now, self.sequence)
            self.assertIsNone(self.commands.poll(self.now))

    def test_source_epoch_change_or_missing_interval_restarts_dwell(self):
        self.hold(ROCK, OPEN, count=3)
        self.assertIsNone(self.feed(ROCK, OPEN, epoch="new"))
        self.assertIsNone(self.feed(ROCK, OPEN, dt=.5, epoch="new"))
        self.assertIsNone(self.feed(ROCK, OPEN, epoch="new"))
        self.assertIsNone(self.feed(ROCK, OPEN, epoch="new"))
        self.assertEqual(self.feed(ROCK, OPEN, epoch="new"), ("pause", ("left",)))

    def test_inhibit_requires_a_fresh_non_v_pose_not_cached_release(self):
        self.feed(OPEN, V)
        self.commands.inhibit()
        self.assertIsNone(self.commands.poll(self.now))
        self.assertEqual(self.hold(V, V, count=8), [None] * 8)
        self.feed(OPEN, V)
        self.assertEqual(self.hold(V, V), [None, None, None, ("engage", ("left", "right"))])

    def test_inhibit_cancels_partial_v_but_never_blocks_a_stop(self):
        self.hold(V, V, count=3)
        self.commands.inhibit()
        self.assertEqual(self.hold(V, V, count=8), [None] * 8)
        self.assertEqual(self.hold(V, ROCK), [None, None, None, ("pause", ("right",))])
        self.assertEqual(self.hold(V, V), [None, None, None, ("engage", ("left", "right"))])

    def test_unknown_or_invalid_data_does_not_rearm_start(self):
        self.hold(V, V)
        for pose in (None, replace(OPEN, confidences=(0.,) * 21), skeleton((90, 75, 0, 140, 140))):
            self.assertEqual(self.hold(pose, V), [None] * 4)
            self.assertEqual(self.hold(V, V), [None] * 4)

    def test_backward_source_sequence_or_timestamp_cannot_confirm(self):
        for sequence, time_offset in ((1, 0), (4, -200_000_000)):
            self.setUp()
            self.hold(ROCK, OPEN, count=3)
            self.now += 100_000_000
            self.samples["left"] = frame("left", ROCK, self.now + time_offset, sequence)
            self.assertIsNone(self.commands.poll(self.now))
            self.sequence = 4
            self.assertEqual(self.hold(ROCK, OPEN), [None, None, None, ("pause", ("left",))])

    def test_invalid_or_future_dated_samples_break_confirmation(self):
        for valid, offset in ((False, 0), (True, 100_000_000)):
            self.setUp()
            self.hold(ROCK, OPEN, count=3)
            self.now += 100_000_000
            self.samples["left"] = frame("left", ROCK, self.now + offset, 4, valid=valid)
            self.assertIsNone(self.commands.poll(self.now))
            self.sequence = 4
            self.assertEqual(self.hold(ROCK, OPEN), [None, None, None, ("pause", ("left",))])

    def test_uint32_source_wrap_preserves_confirmation(self):
        self.sequence = (1 << 32) - 3
        self.assertIsNone(self.feed(ROCK, OPEN))
        self.assertIsNone(self.feed(ROCK, OPEN))
        self.sequence = -1
        self.assertIsNone(self.feed(ROCK, OPEN))
        self.assertEqual(self.feed(ROCK, OPEN), ("pause", ("left",)))

    def test_two_sources_and_positive_finite_timeouts_are_required(self):
        for sources, options in (({}, {}), ({"left": lambda: None}, {}),
                                 (self.commands.sources, {"hold_s": 0}),
                                 (self.commands.sources, {"timeout_s": float("nan")})):
            with self.assertRaises(ValueError):
                GestureCommands(sources, **options)

    def test_live_poll_timestamps_after_both_read_callbacks(self):
        clock, sequence = [1_000_000_000], [0]

        def read(side):
            clock[0] += 100_000
            return frame(side, V, clock[0], sequence[0])

        commands = GestureCommands({s: lambda s=s: read(s) for s in ("left", "right")})
        actions = []
        with patch("bimanual_teleop.control.hand.gesture.time.monotonic_ns", side_effect=lambda: clock[0]):
            for _ in range(4):
                clock[0] += 100_000_000
                sequence[0] += 1
                actions.append(commands.poll())
        self.assertEqual(actions, [None, None, None, ("engage", ("left", "right"))])

    def test_start_waits_for_fresh_confirmation_after_readiness(self):
        for _ in range(8):
            self.assertIsNone(self.feed(V, V, start_ready=False))
        self.assertIsNone(self.commands.poll(self.now, start_ready=True))
        self.assertEqual(self.hold(V, V), [None, None, None, ("engage", ("left", "right"))])
        self.assertEqual(self.hold(V, V), [None] * 4)

    def test_readiness_cannot_override_fault_inhibit_or_block_stop(self):
        self.commands.inhibit()
        for _ in range(4):
            self.assertIsNone(self.feed(V, V, start_ready=False))
        self.assertEqual(self.hold(V, V), [None] * 4)
        self.assertEqual([self.feed(V, ROCK, start_ready=False) for _ in range(4)],
                         [None, None, None, ("pause", ("right",))])


class HomeGestureTests(unittest.TestCase):
    def setUp(self):
        self.samples = {"left": None, "right": None}
        self.now, self.sequence = 1_000_000_000, 0
        self.commands = GestureCommands({s: lambda s=s: self.samples[s] for s in self.samples})

    def feed(self, left=OPEN, right=OPEN, *, ready=True):
        self.now += 100_000_000
        self.sequence += 1
        self.samples = {s: None if pose is None else frame(s, pose, self.now, self.sequence)
                        for s, pose in (("left", left), ("right", right))}
        return self.commands.poll(self.now, start_ready=False, home_ready=ready)

    def test_pause_requires_fresh_release_and_one_second_overlap(self):
        for _ in range(15):
            self.assertIsNone(self.feed(ready=False))
        for _ in range(15):
            self.assertIsNone(self.feed())
        self.feed(V, V)
        for _ in range(10):
            self.assertIsNone(self.feed())
        self.assertEqual(self.feed(), ("home", ("left", "right")))
        for _ in range(15):
            self.assertIsNone(self.feed())
        self.feed(V, V)
        actions = [self.feed() for _ in range(11)]
        self.assertEqual(actions[-1], ("home", ("left", "right")))

    def test_missing_repeated_or_stale_frames_cannot_complete_home(self):
        self.feed(V, V)
        for _ in range(9):
            self.feed()
        self.feed(OPEN, None)
        for _ in range(5):
            self.assertIsNone(self.feed())
        for _ in range(12):
            self.now += 100_000_000
            self.assertIsNone(self.commands.poll(self.now, start_ready=False, home_ready=True))

    def test_one_rock_hand_can_open_to_home_while_other_hand_stays_open(self):
        for _ in range(4):
            action = self.feed(ROCK, OPEN, ready=False)
        self.assertEqual(action, ("pause", ("left",)))
        self.commands.inhibit()
        actions = [self.feed() for _ in range(11)]
        self.assertEqual(actions, [None] * 10 + [("home", ("left", "right"))])
        for _ in range(15):
            self.assertIsNone(self.feed())

    def test_stop_has_priority_and_home_inhibit_requires_release_again(self):
        self.feed(V, V)
        for _ in range(9):
            self.feed()
        self.commands.inhibit()
        for _ in range(12):
            self.assertIsNone(self.feed())
        actions = [self.feed(ROCK, OPEN) for _ in range(4)]
        self.assertEqual(actions[-1], ("pause", ("left",)))


if __name__ == "__main__":
    unittest.main()
