"""Leveled-headset coordinates, using independent quaternion expectations."""

from dataclasses import replace
import math
import unittest

from test_quest import sample as quest_sample
from test_quest_tianji_mapping import SIDES, T0, axis_quat, product, robot
from bimanual_teleop.control.arm.mapping import QuestTianjiMapper, orientation_distance
from bimanual_teleop.control.arm.quest import operator_input


def flu_sample(*, head_position=(1., 2., 3.), head_rotation=(0., 0., 0., 1.), origin=0):
    sample = quest_sample(received_ns=T0, origin=origin)
    frame = sample.payload
    return replace(sample, payload=replace(frame,
        head=replace(frame.head, position_m=head_position, orientation_xyzw=head_rotation),
        left=replace(frame.left, position_m=(4., 6., 8.),
                     orientation_xyzw=product(axis_quat("x", 30), axis_quat("z", 20))),
        right=replace(frame.right, position_m=(-2., 3., 4.),
                      orientation_xyzw=product(axis_quat("y", -25), axis_quat("x", 40)))))


class CoordinateFrameTests(unittest.TestCase):
    def assertQuaternion(self, actual, expected):
        self.assertAlmostEqual(abs(sum(a*b for a, b in zip(actual, expected))), 1., places=8)

    def assertPosition(self, actual, expected):
        for a, b in zip(actual, expected):
            self.assertAlmostEqual(a, b)

    def assertSameWrists(self, actual, expected):
        for side in SIDES:
            a, b = actual.wrists[side].payload.pose, expected.wrists[side].payload.pose
            self.assertPosition(a.position_m, b.position_m)
            self.assertQuaternion(a.orientation_xyzw, b.orientation_xyzw)

    def test_default_headset_yaw_90_rotates_horizontal_axes_and_preserves_up(self):
        sample = flu_sample(head_rotation=axis_quat("z", 90))
        actual = operator_input(sample)
        for side, expected in (("left", (4., -3., 5.)), ("right", (1., 3., 1.))):
            pose = actual.wrists[side].payload.pose
            self.assertPosition(pose.position_m, expected)
            self.assertQuaternion(pose.orientation_xyzw,
                                  product(axis_quat("z", -90), getattr(sample.payload, side).orientation_xyzw))

    def test_pitch_and_roll_do_not_change_headset_reference(self):
        sample = flu_sample(head_rotation=axis_quat("z", 37))
        expected = operator_input(sample)
        for pitch, roll in ((25, 60), (-55, -35), (0, 180)):
            with self.subTest(pitch=pitch, roll=roll):
                rotation = product(axis_quat("z", 37), product(axis_quat("y", pitch), axis_quat("x", roll)))
                tilted = replace(sample, payload=replace(sample.payload,
                    head=replace(sample.payload.head, orientation_xyzw=rotation)))
                self.assertSameWrists(operator_input(tilted), expected)

    def test_common_translation_and_world_yaw_keep_relative_targets_unchanged(self):
        sample = flu_sample(head_rotation=product(axis_quat("z", 20), axis_quat("y", 25)))
        expected = operator_input(sample)
        measured = robot()
        mapper = QuestTianjiMapper("test-profile")
        mapper.reset_reference(expected, measured)
        for yaw in (0, 90):
            with self.subTest(yaw=yaw):
                poses = {}
                for name in ("head", *SIDES):
                    pose = getattr(sample.payload, name)
                    x, y, z = pose.position_m
                    rotated = (-y, x, z) if yaw == 90 else (x, y, z)
                    poses[name] = replace(pose,
                        position_m=tuple(p+t for p, t in zip(rotated, (.3, -.5, .7))),
                        orientation_xyzw=product(axis_quat("z", yaw), pose.orientation_xyzw))
                actual = operator_input(replace(sample, payload=replace(sample.payload, **poses)))
                self.assertSameWrists(actual, expected)
                target = mapper.compute(actual, measured, now_monotonic_ns=T0)
                for side in SIDES:
                    self.assertPosition(target.tool_poses[side].position_m, measured.tool_poses[side].payload.position_m)
                    self.assertQuaternion(target.tool_poses[side].orientation_xyzw,
                                          measured.tool_poses[side].payload.orientation_xyzw)

    def test_head_translation_alone_produces_opposite_relative_motion(self):
        sample = flu_sample()
        measured = robot()
        mapper = QuestTianjiMapper("test-profile")
        mapper.reset_reference(operator_input(sample), measured)
        moved = replace(sample, payload=replace(sample.payload,
            head=replace(sample.payload.head, position_m=(1.01, 1.98, 3.03))))
        target = mapper.compute(operator_input(moved), measured, now_monotonic_ns=T0)
        for side, delta in (("left", (-.01, .03, .02)), ("right", (-.01, -.03, -.02))):
            initial = measured.tool_poses[side].payload
            self.assertPosition(target.tool_poses[side].position_m,
                                tuple(p+d for p, d in zip(initial.position_m, delta)))
            self.assertQuaternion(target.tool_poses[side].orientation_xyzw, initial.orientation_xyzw)

    def test_world_coordinates_do_not_depend_on_head_motion_or_tracking(self):
        sample = flu_sample()
        expected = operator_input(sample, coordinate_frame="world")
        for side in SIDES:
            original = getattr(sample.payload, side)
            pose = expected.wrists[side].payload.pose
            self.assertEqual(pose.position_m, original.position_m)
            self.assertEqual(pose.orientation_xyzw, original.orientation_xyzw)
        moved = replace(sample, header=replace(sample.header, valid=False), payload=replace(sample.payload,
            head=replace(sample.payload.head, position_m=(10., 20., 30.),
                         orientation_xyzw=axis_quat("z", 90), location_flags=0)))
        actual = operator_input(moved, coordinate_frame="world")
        self.assertSameWrists(actual, expected)
        self.assertTrue(all(s.header.valid for s in actual.wrists.values()))
        with self.assertRaisesRegex(ValueError, "head tracking"):
            operator_input(moved)

    def test_yaw_wrap_is_continuous_in_position_and_full_orientation(self):
        sample = flu_sample(head_rotation=axis_quat("z", 179.9), head_position=(0., 0., 0.))
        sample = replace(sample, payload=replace(sample.payload,
            left=replace(sample.payload.left, position_m=(1., 0., 2.))))
        before = operator_input(sample).wrists["left"].payload.pose
        sample = replace(sample, payload=replace(sample.payload,
            head=replace(sample.payload.head, orientation_xyzw=axis_quat("z", -179.9))))
        after = operator_input(sample).wrists["left"].payload.pose
        self.assertPosition(before.position_m, (-0.9999984769132877, -0.0017453283658983, 2.))
        self.assertPosition(after.position_m, (-0.9999984769132877, 0.0017453283658983, 2.))
        self.assertAlmostEqual(orientation_distance(before.orientation_xyzw, after.orientation_xyzw), math.radians(.2))

    def test_near_vertical_head_forward_rejects_only_headset_mode(self):
        for projection in (0., .5e-6):
            with self.subTest(projection=projection):
                sample = flu_sample(head_rotation=axis_quat("y", math.degrees(math.acos(projection))))
                with self.assertRaisesRegex(ValueError, "yaw.*vertical"):
                    operator_input(sample)
                operator_input(sample, coordinate_frame="world")
        operator_input(flu_sample(head_rotation=axis_quat("y", math.degrees(math.acos(2e-6)))))

    def test_source_timing_origin_and_physical_controller_names_are_preserved(self):
        sample = flu_sample(origin=7)
        for frame in ("headset", "world"):
            with self.subTest(frame=frame):
                result = operator_input(sample, sides=("right",), coordinate_frame=frame)
                self.assertEqual(set(result.wrists), {"right"})
                wrist = result.wrists["right"]
                self.assertEqual(wrist.header, sample.header)
                self.assertEqual(wrist.payload.pose.child_frame, sample.payload.right.child_frame)
                expected_parent = ("quest_headset_flu/test-session/7" if frame == "headset"
                                   else sample.payload.right.parent_frame)
                self.assertEqual(wrist.payload.pose.parent_frame, expected_parent)
        with self.assertRaisesRegex(ValueError, "coordinate_frame"):
            operator_input(sample, coordinate_frame="grip")


if __name__ == "__main__":
    unittest.main()
