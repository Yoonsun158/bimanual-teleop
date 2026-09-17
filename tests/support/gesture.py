"""Synthetic and recorded glove gesture samples."""

import json
import math
from pathlib import Path

from bimanual_teleop.devices.wuji.adapter import SKELETON_NAMES
from bimanual_teleop.types import HandSkeleton, Sample, SampleHeader, SampleRef

def skeleton(bends=(90, 0, 140, 140, 0), *, mcp=(0,) * 5):
    points = [(0., 0., 0.)]
    for finger, bend in enumerate(bends):
        point = (.02 * (finger - 2), .025, 0.)
        points.append(point)
        length = math.hypot(point[0], point[1])
        axis = (point[0] / length, point[1] / length)
        for angle in (0, bend / 2, bend):
            angle = math.radians(angle + mcp[finger])
            point = (point[0] + .02 * math.cos(angle) * axis[0],
                     point[1] + .02 * math.cos(angle) * axis[1], point[2] + .02 * math.sin(angle))
            points.append(point)
    return HandSkeleton("l_wrist", SKELETON_NAMES, tuple(points), (1.,) * 21, ())


ROCK = skeleton()


V = skeleton((90, 0, 0, 140, 140))


OPEN = skeleton((0,) * 5)


def frame(side, pose, now, sequence, *, valid=True, epoch="test"):
    return Sample(SampleHeader(SampleRef(f"{side}/skeleton", epoch, sequence), now,
                              valid, source_sequence=sequence), pose)


def recorded_pose(name):
    fixture = json.loads((Path(__file__).resolve().parents[1] / "fixtures/wuji_gestures.json").read_text())[name]
    p = fixture["sample"]["payload"]
    return HandSkeleton(p["frame"], tuple(p["joint_names"]), tuple(map(tuple, p["positions_m"])),
                        tuple(p["confidences"]), ())
