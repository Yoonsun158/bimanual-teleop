"""Pose and mapping fixtures; expected rotations computed independently."""

import math

from bimanual_teleop.types import (
    OperatorInput, Pose, RobotState, RobotTarget, Sample, SampleHeader, SampleRef, TrackedPose,
)

SIDES = ("left", "right")


T0 = 1_000_000_000


def axis_quat(axis, degrees):
    half = math.radians(degrees) / 2
    values = [0.0, 0.0, 0.0, math.cos(half)]
    values["xyz".index(axis)] = math.sin(half)
    return tuple(values)


def product(a, b):
    """Independent quaternion calculation for expected noncommuting rotations."""
    x, y, z, w = a
    X, Y, Z, W = b
    return (w*X+x*W+y*Z-z*Y, w*Y-x*Z+y*W+z*X,
            w*Z+x*Y-y*X+z*W, w*W-x*X-y*Y-z*Z)


def operator(positions=None, rotations=None, *, sequence=0, received_ns=T0, origin="origin-0"):
    positions = positions or {"left": (1.3, -.8, .7), "right": (-.2, .6, .5)}
    rotations = rotations or {side: axis_quat("y", -90) for side in SIDES}
    header = SampleHeader(SampleRef("quest.poses", f"session/{origin}", sequence), received_ns, True)
    wrists = {side: Sample(header, TrackedPose(
        Pose(f"quest_local_flu/session/{origin}", f"quest_{side}_grip_flu", positions[side], rotations[side]),
        True, True, True, True)) for side in SIDES}
    return OperatorInput(wrists)


def robot(positions=None, rotations=None):
    positions = positions or {"left": (.4, -.2, .5), "right": (.3, .2, .6)}
    rotations = rotations or {"left": product(axis_quat("y", -40), axis_quat("z", 15)),
                              "right": product(axis_quat("x", 25), axis_quat("y", 40))}
    header = SampleHeader(SampleRef("tianji.feedback", "robot-session", 20), T0, True)
    return RobotState({side: Sample(header, Pose(f"tianji_{side}_base", f"tianji_{side}_flange",
                                                   positions[side], rotations[side])) for side in SIDES})


def target(poses, *, sequence=1, created_ns=T0, expires_ns=T0+100_000_000):
    return RobotTarget(f"goal-{sequence}", dict(poses), (SampleRef("quest.poses", "session", sequence),),
                       created_ns, expires_ns, "verified-profile")
