"""Small nonblocking device observer; no encoding, FK, or disk access here."""

from dataclasses import dataclass
import math
from queue import Full

from bimanual_teleop.types import CommandEvent, CommandStatus, Event, JointState

SIDES = ("left", "right")
STATE_STREAMS = tuple(f"{kind}/{side}" for kind in ("arms", "hands") for side in SIDES)
COMMAND_STREAMS = tuple(f"{kind}/{side}" for kind in ("arm_commands", "hand_commands") for side in SIDES)


@dataclass(frozen=True)
class Record:
    stream: str
    time_ns: int
    sequence: int
    values: dict


def pose_values(pose):
    return (*pose.position_m, *pose.orientation_xyzw)


def finite(values, size):
    return values is not None and len(values) == size and all(
        v is not None and math.isfinite(v) for v in values)


class CaptureChannel:
    """Spawn-compatible shared state; failure signaling is separate from data."""

    def __init__(self, context, capacity=4096):
        self.queue = context.Queue(capacity)
        self.errors = context.Queue(16)
        self.failed = context.Event()
        self.active = context.Value("b", False, lock=False)
        self.generation = context.Value("q", 0, lock=False)
        self.latest_ns = context.Array("q", 8, lock=False)
        self.sent = context.Array("q", 8, lock=False)
        self.consumed = context.Array("q", 8, lock=False)
        self.inflight = context.Array("b", 8, lock=False)

    def fail(self, reason):
        if not self.failed.is_set():
            try:
                self.errors.put_nowait(str(reason))
            except (Full, OSError, ValueError):
                pass
            self.failed.set()


class RecorderSink:
    def __init__(self, channel, state_hz=200.):
        self.channel = channel
        self.period_ns = round(1e9 / state_hz)
        self._next_ns = {}
        self._sequences = {}
        self._pending = {}
        self._command_sequence = 0

    def _put(self, record, *, state=False):
        streams = STATE_STREAMS + COMMAND_STREAMS
        index = streams.index(record.stream)
        self.channel.latest_ns[index] = record.time_ns
        if not self.channel.active.value or self.channel.failed.is_set():
            return
        if state:
            next_ns = self._next_ns.get(record.stream, record.time_ns)
            if record.time_ns < next_ns:
                return
            self._next_ns[record.stream] = next_ns + (
                (record.time_ns - next_ns) // self.period_ns + 1) * self.period_ns
        self.channel.inflight[index] = True
        try:
            if not self.channel.active.value:
                return
            self.channel.queue.put_nowait((self.channel.generation.value, record))
            self.channel.sent[index] += 1
        except (Full, OSError, ValueError) as error:
            self.channel.fail(f"录制队列无法接收数据：{type(error).__name__}")
        finally:
            self.channel.inflight[index] = False

    def _invalid(self, stream):
        self.channel.latest_ns[STATE_STREAMS.index(stream)] = 0
        if self.channel.active.value:
            self.channel.fail(f"采集数据无效：{stream}")

    def try_publish(self, sample):
        try:
            stream = sample.header.ref.stream
            now = sample.header.received_monotonic_ns
            if stream == "tianji.feedback":
                for side, arm in sample.payload.arms.items():
                    name = f"arms/{side}"
                    sequence = arm.source_sequence
                    if self._sequences.get(name) == sequence:
                        continue
                    self._sequences[name] = sequence
                    if not finite(arm.joints.position_rad, 7) or not finite(arm.wrench, 6):
                        self._invalid(name)
                        continue
                    self._put(Record(name, now, sequence, {
                        "joint_pos": tuple(arm.joints.position_rad), "wrench": tuple(arm.wrench)}), state=True)
            elif isinstance(sample.payload, JointState) and stream in (
                    "wuji_left_hand/joints", "wuji_right_hand/joints"):
                side = "left" if stream == "wuji_left_hand/joints" else "right"
                name = f"hands/{side}"
                identity = (sample.header.ref.epoch, sample.header.source_sequence
                            if sample.header.source_sequence is not None else sample.header.ref.sequence)
                if self._sequences.get(name) == identity:
                    return True
                self._sequences[name] = identity
                if not finite(sample.payload.position_rad, 20) or not sample.header.valid:
                    self._invalid(name)
                else:
                    self._put(Record(name, now, sample.header.source_sequence
                        if sample.header.source_sequence is not None else sample.header.ref.sequence,
                        {"joint_pos": tuple(sample.payload.position_rad)}), state=True)
        except Exception as error:
            if self.channel.active.value:
                self.channel.fail(f"采集反馈失败：{error}")
        # Recorder failures stop the episode via the UI, never latch a device observer fault.
        return True

    def try_event(self, event):
        try:
            if isinstance(event, Event) and event.kind == "tianji_command_submitted":
                command = event.details["command"]
                self._command_sequence += 1
                for side, joints in command.payload.targets.items():
                    pose = command.payload.requested_cartesian_targets[side]
                    self._put(Record(f"arm_commands/{side}", event.observed_monotonic_ns,
                        self._command_sequence, {"joint_pos": tuple(joints), "eef_pose": pose_values(pose)}))
            elif isinstance(event, Event) and event.kind == "wuji_command":
                command = event.details["command"]
                self._pending[command.command_id] = command
                if len(self._pending) > 32:
                    self._pending.pop(next(iter(self._pending)))
            elif isinstance(event, CommandEvent):
                command = self._pending.pop(event.command_id, None)
                if command is not None and event.status == CommandStatus.ACCEPTED:
                    side = next(s for s in SIDES if s in command.device_id)
                    self._command_sequence += 1
                    self._put(Record(f"hand_commands/{side}", event.observed_monotonic_ns,
                        self._command_sequence, {"joint_pos": tuple(command.payload.position_rad)}))
        except Exception as error:
            if self.channel.active.value:
                self.channel.fail(f"采集控制目标失败：{error}")
        return True
