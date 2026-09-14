"""Sample-driven commands: both hands V to start, either hand horns to stop."""

from __future__ import annotations

from dataclasses import dataclass, field
import math
import time

from bimanual_teleop.devices.wuji.adapter import SKELETON_NAMES
from bimanual_teleop.types import HandSkeleton


_ROCK = ("bent", "straight", "bent", "bent", "straight")
_V = ("bent", "straight", "straight", "bent", "bent")
_V_HINTS = ("拇指需收拢", "食指需伸直", "中指需伸直", "无名指需收拢", "小指需收拢")


def _geometry(skeleton):
    if (not isinstance(skeleton, HandSkeleton) or skeleton.joint_names != SKELETON_NAMES
            or len(skeleton.positions_m) != 21 or len(skeleton.confidences) != 21
            or any(len(p) != 3 or not all(map(math.isfinite, p)) for p in skeleton.positions_m)):
        return None, "骨架数据无效"
    if any(not math.isfinite(c) or not .5 <= c <= 1. for c in skeleton.confidences):
        return None, "骨架置信度低"
    bends, reaches = [], [None]
    for finger, start in enumerate((1, 5, 9, 13, 17)):
        points = skeleton.positions_m[start:start + 4]
        bones = [tuple(b - a for a, b in zip(p, q)) for p, q in zip(points, points[1:])]
        lengths = [math.sqrt(sum(x * x for x in bone)) for bone in bones]
        if any(not math.isfinite(length) or length < 1e-6 for length in lengths):
            return None, "骨架骨长无效"
        cosines = [sum(a * b for a, b in zip(bones[i], bones[i + 1]))
                   / (lengths[i] * lengths[i + 1]) for i in (0, 1)]
        bends.append(sum(math.degrees(math.acos(max(-1., min(1., c)))) for c in cosines))
        if finger:
            axis = tuple(a - b for a, b in zip(points[0], skeleton.positions_m[0]))
            distance = math.sqrt(sum(x * x for x in axis))
            if not math.isfinite(distance) or distance < 1e-6:
                return None, "骨架掌指方向无效"
            reaches.append(sum((tip - base) * x for tip, base, x in zip(points[-1], points[0], axis))
                           / (distance * sum(lengths)))
    return {"bend_deg": bends, "reach": reaches}, ""


def _bend_poses(bends):
    return ["straight" if bend <= 35 else "bent" if bend >= (50 if finger == 0 else 70)
            else "uncertain" for finger, bend in enumerate(bends)]


def _finger_poses(skeleton):
    """Legacy inter-bone classification retained for verified rock gestures."""
    features, _ = _geometry(skeleton)
    return None if features is None else _bend_poses(features["bend_deg"])


def _matches(fingers, expected):
    if fingers is None:
        return None
    if any(actual not in (wanted, "uncertain") for actual, wanted in zip(fingers, expected)):
        return False
    return None if "uncertain" in fingers else True


def _classify(skeleton):
    features, problem = _geometry(skeleton)
    if features is None:
        return (None, None), problem, {}
    fingers = _bend_poses(features["bend_deg"])
    rock = _matches(fingers, _ROCK)
    # Projection includes MCP flexion, which inter-bone angles alone omit.
    v_fingers = [fingers[0]] + ["straight" if reach >= .75 else "bent" if reach <= .5
                              else "uncertain" for reach in features["reach"][1:]]
    v = False if rock is True else _matches(v_fingers, _V)
    detail = ("摇滚已识别" if rock is True else "V已识别" if v is True else
              "、".join(hint for pose, expected, hint in zip(v_fingers, _V, _V_HINTS) if pose != expected))
    return (rock, v), detail, features


def rock_gesture(skeleton: HandSkeleton) -> bool | None:
    """True for horns, False for a clear different pose, None if uncertain."""
    return _matches(_finger_poses(skeleton), _ROCK)


def v_gesture(skeleton: HandSkeleton) -> bool | None:
    """True when only the index and middle fingers extend away from the palm."""
    return _classify(skeleton)[0][1]


@dataclass
class _Hand:
    identity: tuple | None = None
    received_ns: int | None = None
    poses: tuple = (None, None)
    since: tuple = (None, None)
    rock_armed: bool = True
    detail: str = "尚无骨架样本"
    features: dict = field(default_factory=dict)

    def invalidate(self, reason):
        self.poses = self.since = (None, None)
        self.detail, self.features = reason, {}


class GestureCommands:
    """Distinct frames confirm a gesture; held commands are emitted only once.

    Starting requires overlapping confirmation from both hands. Stopping is
    independent per hand, so an unavailable or ambiguous other hand cannot block it.
    """

    def __init__(self, sources, *, timeout_s=.25, hold_s=.3):
        if (set(sources) != {"left", "right"}
                or any(not math.isfinite(v) or v <= 0 for v in (timeout_s, hold_s))):
            raise ValueError("Both hand sources and positive finite timeouts are required")
        self.sources = dict(sources)
        self.timeout_ns, self.hold_ns = (round(v * 1e9) for v in (timeout_s, hold_s))
        self._hands = {side: _Hand() for side in sources}
        self._start_armed = True

    def inhibit(self):
        """Require a fresh, clearly non-V frame before allowing another start."""
        self._start_armed = False
        for hand in self._hands.values():
            hand.since = (hand.since[0], None)

    def poll(self, now_ns=None, *, start_ready=True):
        # The snapshot can be newer than a timestamp taken before reading it.
        samples = {side: read() for side, read in self.sources.items()}
        now_ns = time.monotonic_ns() if now_ns is None else now_ns
        for side, sample in samples.items():
            hand = self._hands[side]
            if sample is None or not sample.header.valid:
                hand.invalidate("尚无有效骨架样本")
                continue
            header = sample.header
            age = now_ns - header.received_monotonic_ns
            if not 0 <= age < self.timeout_ns:
                hand.invalidate("样本过期" if age >= 0 else "样本时间戳超前")
                continue
            identity = (header.ref.epoch, header.source_sequence
                        if header.source_sequence is not None else header.ref.sequence)
            if hand.identity == identity:
                continue
            previous = hand.identity
            if (previous is not None and previous[0] == identity[0]
                    and not 0 < (identity[1] - previous[1]) % (1 << 32) < (1 << 31)):
                hand.invalidate("帧序号回退")
                continue
            hand.identity = identity
            received = header.received_monotonic_ns
            if hand.received_ns is not None and received <= hand.received_ns:
                hand.invalidate("样本时间戳回退")
                continue
            continuous = (previous is not None and previous[0] == identity[0]
                          and received - hand.received_ns < self.timeout_ns)
            poses, hand.detail, hand.features = _classify(sample.payload)
            hand.since = tuple(None if pose is not True else
                               since if continuous and old is True and since is not None else received
                               for pose, old, since in zip(poses, hand.poses, hand.since))
            hand.poses, hand.received_ns = poses, received
            if poses[0] is False:
                hand.rock_armed = True
            if poses[1] is False:
                self._start_armed = True

        if not start_ready:
            for hand in self._hands.values():
                hand.since = (hand.since[0], None)

        sides = tuple(side for side, hand in self._hands.items()
                      if hand.rock_armed and hand.poses[0] is True and hand.since[0] is not None
                      and hand.received_ns - hand.since[0] >= self.hold_ns)
        if sides:
            for side in sides:
                self._hands[side].rock_armed = False
            return "pause", sides
        if start_ready and self._start_armed and self._v_hold_ns(now_ns) >= self.hold_ns:
            self._start_armed = False
            return "engage", tuple(self.sources)
        return None

    def _v_hold_ns(self, now_ns):
        if any(h.poses[1] is not True or h.since[1] is None
               or not 0 <= now_ns - h.received_ns < self.timeout_ns for h in self._hands.values()):
            return 0
        return max(0, min(h.received_ns for h in self._hands.values())
                   - max(h.since[1] for h in self._hands.values()))

    def status(self, now_ns):
        """Describe cached recognition without reading samples or advancing dwell."""
        hands = {}
        for side, hand in self._hands.items():
            rock, v = hand.poses
            detail = hand.detail
            if hand.received_ns is not None:
                age = now_ns - hand.received_ns
                if not 0 <= age < self.timeout_ns:
                    rock = v = None
                    detail = "样本过期" if age >= 0 else "样本时间戳超前"
            hands[side] = {"v": v, "rock": rock, "detail": detail,
                           "features": {key: list(value) for key, value in hand.features.items()}}
        return {"hands": hands, "start_armed": self._start_armed,
                "hold_ms": self._v_hold_ns(now_ns) / 1e6, "required_hold_ms": self.hold_ns / 1e6}
