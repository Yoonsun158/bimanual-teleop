"""USB acquisition from the project's native Quest APK; importing opens nothing.

The APK queries OpenXR LOCAL space. Both parent and child axes are changed from
right/up/back to forward/left/up here. Timestamps are query times, not optical
capture times; host arrival alone does not establish source freshness.
"""

from __future__ import annotations

from dataclasses import dataclass
import json
import logging
import math
import subprocess
import threading
import time
import uuid
from typing import Mapping

from bimanual_teleop.devices.interfaces import RealtimeObserver
from bimanual_teleop.types import (
    Event, Health, Quaternion, Sample, SampleHeader, SampleRef, SourceTime, Vector3,
)

PACKAGE = "org.bimanual.questcapture"
ACTIVITY = f"{PACKAGE}/.MainActivity"
LOG_TAG = "QuestCapture"
POWER_OVERRIDE_ACTION = "com.oculus.vrpowermanager.prox_close"
POWER_RESTORE_ACTION = "com.oculus.vrpowermanager.automation_disable"
STREAM = "quest.poses"
SILENCE_TIMEOUT_NS = 1_000_000_000
RECOVERABLE_ISSUES = frozenset({
    "malformed", "out_of_order", "reference_metadata_gap", "reference_time_mismatch",
})
LOGGER = logging.getLogger(__name__)


@dataclass(frozen=True)
class QuestPose:
    """A partially available pose, retaining the original OpenXR location flags."""

    parent_frame: str
    child_frame: str
    position_m: Vector3 | None
    orientation_xyzw: Quaternion | None
    location_flags: int
    active: bool | None

    @property
    def position_valid(self) -> bool:
        return bool(self.location_flags & 2)

    @property
    def orientation_valid(self) -> bool:
        return bool(self.location_flags & 1)

    @property
    def position_tracked(self) -> bool:
        return bool(self.location_flags & 8)

    @property
    def orientation_tracked(self) -> bool:
        return bool(self.location_flags & 4)

    @property
    def valid(self) -> bool:
        return self.location_flags & 3 == 3

    @property
    def tracked(self) -> bool:
        return self.location_flags & 12 == 12


@dataclass(frozen=True)
class QuestFrame:
    """Three poses queried at one source time; values remain SDK estimates."""

    session: str
    sequence: int
    origin: int
    query_monotonic_ns: int
    xr_time: int
    send_monotonic_ns: int
    session_state: int
    refresh_hz: float
    head: QuestPose
    left: QuestPose
    right: QuestPose


def _integer(value: object, name: str) -> int:
    if type(value) is not int or value < 0:
        raise ValueError(f"{name} must be a nonnegative integer")
    return value


def _number(value: object, name: str) -> float:
    if type(value) not in (int, float) or not math.isfinite(value):
        raise ValueError(f"{name} must be a finite number")
    return float(value)


def _pose(data: object, parent: str, child: str, head: bool) -> QuestPose:
    if not isinstance(data, dict):
        raise ValueError(f"{child} must be an object")
    flags = _integer(data["flags"], "flags")
    active = data["active"]
    if (head and active is not None) or (not head and type(active) is not bool):
        raise ValueError("active must be null for head and boolean for controllers")
    values = []
    for key, count, valid_bit in (("p", 3, 2), ("q", 4, 1)):
        raw = data[key]
        if not flags & valid_bit:
            if raw is not None:
                raise ValueError(f"invalid {key} must be null")
            values.append(None)
            continue
        if not isinstance(raw, list) or len(raw) != count:
            raise ValueError(f"valid {key} must contain {count} values")
        components = tuple(_number(x, key) for x in raw)
        if key == "q" and not math.isclose(sum(x * x for x in components), 1, abs_tol=0.001):
            raise ValueError("q must be a unit quaternion")
        converted = (-components[2], -components[0], components[1])
        values.append(converted if key == "p" else (*converted, components[3]))
    return QuestPose(parent, child, values[0], values[1], flags, active)


def _check_event(kind: str, details: dict[str, object]) -> None:
    if kind == "reference_space_change":
        _integer(details["origin"], "origin")
        _integer(details["change_time"], "change_time (XrTime)")
        available = details["pose_valid"]
        if type(available) is not bool or not isinstance(details["pose_in_previous_space"], dict):
            raise ValueError("reference change requires pose_valid and pose_in_previous_space")
        _pose({**details["pose_in_previous_space"], "flags": 3 if available else 0, "active": None},
              "", "reference change", True)
    elif kind == "session_state":
        if _integer(details["state"], "state") > 8:
            raise ValueError("invalid XR session state")
        _integer(details["time"], "time (XrTime)")
    elif kind == "refresh_rate":
        if _number(details["from_hz"], "from_hz") < 0 or _number(details["to_hz"], "to_hz") <= 0:
            raise ValueError("invalid display refresh rate")
    elif kind == "error":
        if not isinstance(details["operation"], str) or not details["operation"] or type(details["result"]) is not int:
            raise ValueError("error requires an operation and integer XrResult")


def decode_message(line: str, received_monotonic_ns: int) -> Sample[QuestFrame] | Event:
    """Decode one v1 JSON line. Malformed data raises ValueError, never fills poses."""
    try:
        data = json.loads(line)
        if not isinstance(data, dict) or type(data.get("v")) is not int or data["v"] != 1:
            raise ValueError("expected Quest protocol v1")
        session = data["session"]
        if not isinstance(session, str) or not session:
            raise ValueError("session must be nonempty")
        if data["type"] == "event":
            kind = data["event"]
            if kind not in {"reference_space_change", "session_state", "refresh_rate", "error"}:
                raise ValueError("unknown Quest event")
            device_ns = _integer(data["device_ns"], "device_ns")
            if not isinstance(data["details"], dict):
                raise ValueError("event details must be an object")
            _check_event(kind, data["details"])
            event_data = {
                "session": session, "device_ns": device_ns, "details": data["details"],
                "device_clock_domain": f"quest/{session}/CLOCK_MONOTONIC",
            }
            if kind in {"reference_space_change", "session_state"}:
                event_data["xr_clock_domain"] = f"quest/{session}/XrTime"
            return Event(f"quest.{kind}", received_monotonic_ns, STREAM, event_data)
        if data["type"] != "frame":
            raise ValueError("unknown Quest message type")
        sequence = _integer(data["seq"], "seq")
        origin = _integer(data["origin"], "origin")
        query_ns = _integer(data["query_ns"], "query_ns")
        xr_time = _integer(data["xr_time"], "xr_time")
        send_ns = _integer(data["send_ns"], "send_ns")
        state = _integer(data["state"], "state")
        refresh_hz = _number(data["refresh_hz"], "refresh_hz")
        if send_ns < query_ns or xr_time == 0 or state > 8 or refresh_hz <= 0:
            raise ValueError("invalid frame timing, session state or refresh rate")
        parent = f"quest_local_flu/{session}/{origin}"
        frame = QuestFrame(
            session, sequence, origin, query_ns, xr_time, send_ns, state, refresh_hz,
            _pose(data["head"], parent, "quest_head_view_flu", True),
            _pose(data["left"], parent, "quest_left_grip_flu", False),
            _pose(data["right"], parent, "quest_right_grip_flu", False),
        )
        valid = all(p.valid for p in (frame.head, frame.left, frame.right))
        return Sample(SampleHeader(
            SampleRef(STREAM, f"{session}/origin-{origin}", sequence),
            received_monotonic_ns, valid,
            SourceTime(query_ns, "ns", f"quest/{session}/CLOCK_MONOTONIC", "xr_query_current_time"),
            sequence,
        ), frame)
    except (KeyError, TypeError, OverflowError) as exc:
        raise ValueError(f"malformed Quest message: {exc}") from exc


def select_usb_serial(output: str, requested: str | None = None) -> str:
    """Select only an authorized USB Quest from `adb devices -l` output."""
    candidates, states = [], []
    for line in output.splitlines():
        fields = line.split()
        if len(fields) < 2 or fields[0] == "List":
            continue
        serial, state = fields[:2]
        states.append(f"{serial}: {state}")
        usb = any(x.startswith("usb:") for x in fields[2:])
        quest = any(x.startswith("model:") and "quest" in x.lower() for x in fields[2:])
        if requested == serial and (state != "device" or not usb or not quest):
            raise RuntimeError(f"{serial} is not an authorized USB Quest: {line}")
        if state == "device" and usb and quest and (requested is None or serial == requested):
            candidates.append(serial)
    if len(candidates) != 1:
        found = ", ".join(states) or "no devices"
        raise RuntimeError(f"Expected one authorized USB Quest; specify serial if several are connected. ADB: {found}")
    return candidates[0]


class QuestSource:
    """One USB/ADB owner with optional nonblocking live observation."""

    def __init__(self, serial: str | None = None, *, keep_awake: bool = True):
        self.serial = serial
        self.keep_awake = keep_awake
        self._power_override_requested = False
        self._session: str | None = None
        self._sink: RealtimeObserver | None = None
        self._latest: Sample[QuestFrame] | None = None
        self._problem: str | None = None
        self._data_problem: str | None = None
        self._data_problem_generation = 0
        self._observer_error: str | None = None
        self._pending_origins: dict[int, int] = {}
        self._last_sequence = -1
        self._last_query_ns = -1
        self._last_origin = 0
        self._session_state = 0
        self._refresh_hz = 0.0
        self._process: subprocess.Popen[str] | None = None
        self._thread: threading.Thread | None = None
        self._running = False
        self._owns_app = False
        self._lock = threading.Lock()
        self._last_issue_warning: tuple[str, str] | None = None
        self._last_issue_warning_ns = 0

    @property
    def metadata(self) -> Mapping[str, object]:
        return {
            "protocol_version": 1, "serial": self.serial, "session": self._session,
            "keep_awake_requested": self.keep_awake,
            "requested_hz": 90, "transport": "adb_usb_logcat",
            "reference_space": "LOCAL", "controller_pose": "grip", "head_pose": "VIEW",
            "axes": "right-handed: x forward, y left, z up",
            "basis_from_openxr": ((0, 0, -1), (-1, 0, 0), (0, 1, 0)),
            "basis_applies_to": "parent and child; p=Bp, R=BRB^T",
            "reference_change_transform_axes": "OpenXR: x right, y up, z back",
            "query_time": "session-scoped device CLOCK_MONOTONIC; not sensor capture time",
            "freshness": "host arrival does not establish source age; clocks are not synchronized",
            "silence_timeout_ns": SILENCE_TIMEOUT_NS,
            "silence_timeout_meaning": "no frame arriving at host; not a source-age guarantee",
            "observer_error": self._observer_error,
        }

    def _adb(self, *args: str) -> subprocess.CompletedProcess[str]:
        return subprocess.run(["adb", *args], capture_output=True, text=True, check=True, timeout=10)

    def start(self, sink: RealtimeObserver | None = None) -> None:
        if self._process is not None:
            raise RuntimeError("QuestSource is already started")
        self.serial = select_usb_serial(self._adb("devices", "-l").stdout, self.serial)
        target = ("-s", self.serial)
        if "package:" not in self._adb(*target, "shell", "pm", "path", PACKAGE).stdout:
            raise RuntimeError(f"Install the project APK {PACKAGE} before starting")
        try:
            existing = self._adb(*target, "shell", "pidof", PACKAGE).stdout.strip()
        except subprocess.CalledProcessError as exc:
            if exc.returncode != 1:
                raise
            existing = ""
        if existing:
            raise RuntimeError(f"{PACKAGE} is already running; stop that session first")
        self._sink, self._session = sink, uuid.uuid4().hex
        self._latest, self._problem = None, None
        self._data_problem, self._data_problem_generation = None, 0
        self._observer_error = None
        self._last_sequence = self._last_query_ns = -1
        self._last_origin = 0
        self._pending_origins.clear()
        self._session_state, self._refresh_hz = 0, 0.0
        self._last_issue_warning, self._last_issue_warning_ns = None, 0
        try:
            if self.keep_awake:
                # Meta's development override keeps XR active off-face. This is
                # separate from Android's KEEP_SCREEN_ON; undo it on close.
                # This wakes the display. Quest 3S sensor lock may still need
                # its physical power button; the override cannot unlock it.
                self._adb(*target, "shell", "input", "keyevent", "KEYCODE_WAKEUP")
                # Mark before sending so a timeout also attempts cleanup.
                self._power_override_requested = True
                self._adb(*target, "shell", "am", "broadcast", "-a", POWER_OVERRIDE_ACTION)
            self._process = subprocess.Popen(
                ["adb", *target, "logcat", "-v", "raw", "-T", "1", f"{LOG_TAG}:I", "*:S"],
                stdin=subprocess.DEVNULL, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                text=True, encoding="utf-8",
            )
            self._running = True
            self._thread = threading.Thread(target=self._read_loop, name="quest-receiver", daemon=True)
            self._thread.start()
            if self._process.poll() is not None:
                raise RuntimeError("ADB log stream exited before APK launch")
            self._owns_app = True
            result = self._adb(*target, "shell", "am", "start", "-n", ACTIVITY,
                               "--es", "session_id", self._session)
            if "Error:" in result.stdout or "Error:" in result.stderr:
                raise RuntimeError(result.stdout + result.stderr)
            if self._process.poll() is not None:
                raise RuntimeError("ADB log stream exited during APK launch")
        except Exception:
            self.close()
            raise

    def _event(self, event: Event) -> None:
        if self._sink is not None:
            error = None
            try:
                if self._sink.try_event(event):
                    return
            except Exception as exc:
                error = exc
            self._observer_failure(error)

    def _observer_failure(self, error: Exception | None = None) -> None:
        with self._lock:
            first = self._observer_error is None
            if first:
                self._observer_error = (f"Quest realtime observer failed: {error}" if error else
                                        "Quest realtime observer rejected live data")
                self._problem = self._observer_error
        if first:
            LOGGER.error(self._observer_error)

    def _issue(self, kind: str, detail: str, received_ns: int, **extra: object) -> None:
        with self._lock:
            if kind in RECOVERABLE_ISSUES:
                self._data_problem = detail
                self._data_problem_generation += 1
            elif kind != "source_gap":
                self._problem = detail
        signature = (kind, detail)
        if (signature != self._last_issue_warning or
                received_ns - self._last_issue_warning_ns >= 5_000_000_000):
            LOGGER.warning("Quest: %s", detail)
            self._last_issue_warning = signature
            self._last_issue_warning_ns = received_ns
        self._event(Event(f"quest.{kind}", received_ns, STREAM, {
            "session": self._session, "detail": detail, **extra,
        }))

    def _process_line(self, line: str, received_ns: int) -> None:
        if not line.strip() or line.startswith("---------"):
            return
        try:
            raw = json.loads(line)
            if isinstance(raw, dict) and isinstance(raw.get("session"), str) and raw["session"] != self._session:
                return
            message = decode_message(line, received_ns)
        except (ValueError, TypeError) as exc:
            self._issue("malformed", str(exc), received_ns)
            return
        if isinstance(message, Event):
            details = message.details["details"]
            if message.kind == "quest.reference_space_change":
                self._pending_origins[details["origin"]] = details["change_time"]
                with self._lock:
                    self._latest = None
            elif message.kind == "quest.session_state":
                self._session_state = details["state"]
            elif message.kind == "quest.refresh_rate":
                self._refresh_hz = details["to_hz"]
            elif message.kind == "quest.error":
                with self._lock:
                    self._problem = f"Quest runtime error: {details}"
                LOGGER.error(self._problem)
            self._event(message)
            return
        frame = message.payload
        with self._lock:
            issue_generation = self._data_problem_generation
        if frame.sequence <= self._last_sequence or frame.query_monotonic_ns <= self._last_query_ns:
            self._issue("out_of_order", "source sequence or query time did not increase", received_ns)
            return
        if frame.sequence != self._last_sequence + 1:
            self._issue("source_gap", "missing source frames", received_ns,
                        first_source_sequence=self._last_sequence + 1,
                        last_source_sequence=frame.sequence - 1)
        effective = sorted((t, origin) for origin, t in self._pending_origins.items() if t <= frame.xr_time)
        if frame.origin != self._last_origin:
            self._event(Event("quest.origin_changed", received_ns, STREAM, {
                "session": frame.session, "previous_origin": self._last_origin, "origin": frame.origin,
                "query_ns": frame.query_monotonic_ns, "xr_time": frame.xr_time,
                "boundary": "first_received_frame_in_new_origin",
            }))
            if frame.origin not in self._pending_origins:
                self._issue("reference_metadata_gap", "origin changed without its reference-space event", received_ns,
                            previous_origin=self._last_origin, origin=frame.origin)
        if effective and frame.origin == effective[-1][1]:
            for _, origin in effective:
                del self._pending_origins[origin]
        elif effective or frame.origin != self._last_origin and frame.origin in self._pending_origins:
            self._issue("reference_time_mismatch", "frame origin does not match effective reference-space event", received_ns,
                        origin=frame.origin, xr_time=frame.xr_time)
        self._last_sequence, self._last_query_ns = frame.sequence, frame.query_monotonic_ns
        self._last_origin = frame.origin
        self._session_state, self._refresh_hz = frame.session_state, frame.refresh_hz
        if self._sink is not None:
            error = None
            try:
                accepted = self._sink.try_publish(message)
            except Exception as exc:
                error = exc
                accepted = False
            if not accepted:
                self._observer_failure(error)
        # Publish to the safety monitor before advertising source readiness. A
        # clean frame recovers acquisition, while the monitor's fault latch still
        # requires explicit re-engagement after malformed data or origin changes.
        with self._lock:
            self._latest = message
            if issue_generation == self._data_problem_generation:
                self._data_problem = None

    def _read_loop(self) -> None:
        assert self._process is not None and self._process.stdout is not None
        last_diagnostic = None
        try:
            for line in self._process.stdout:
                received_ns = time.monotonic_ns()
                if not self._running:
                    break
                if line.lstrip().startswith(("adb:", "logcat:", "error:")):
                    last_diagnostic = line.strip()[:512]
                self._process_line(line, received_ns)
            if self._running:
                code = self._process.poll()
                detail = "ADB log stream ended"
                if code is not None:
                    detail += f" (exit code {code})"
                if last_diagnostic:
                    detail += f": {last_diagnostic}"
                self._issue("disconnected", detail, time.monotonic_ns(),
                            exit_code=code, transport_diagnostic=last_diagnostic)
        except (OSError, UnicodeError) as exc:
            if self._running:
                self._issue("disconnected", f"ADB log stream failed: {exc}", time.monotonic_ns())

    def get_latest(self) -> Sample[QuestFrame] | None:
        with self._lock:
            return self._latest

    def health(self, *, sides: tuple[str, ...] = ("left", "right"), require_head: bool = True) -> Health:
        if not sides or len(set(sides)) != len(sides) or any(side not in ("left", "right") for side in sides):
            raise ValueError("Quest health sides must be left, right, or both")
        with self._lock:
            sample = self._latest
            problem = self._observer_error or self._problem or self._data_problem
        now = time.monotonic_ns()
        if not self._running:
            return Health(False, now, "closed")
        if problem:
            return Health(False, now, problem)
        if self._pending_origins:
            return Health(False, now, "Quest reference space is changing; wait for a new origin frame, then re-engage")
        if sample is None:
            return Health(False, now, "waiting for Quest frames")
        if now - sample.header.received_monotonic_ns > SILENCE_TIMEOUT_NS:
            return Health(False, now, "no Quest frame arrived for 1 second; host stream is silent")
        frame = sample.payload
        if self._session_state != 5:
            return Health(False, now, "Quest XR session not focused; close the headset system menu")
        for side in (("head", *sides) if require_head else sides):
            pose = getattr(frame, side)
            if not pose.valid or not pose.tracked or pose.active is False:
                return Health(False, now, f"Quest {side} tracking unavailable (flags={pose.location_flags}, "
                              f"active={pose.active}); wake the controller and keep it visible to the headset")
        if not math.isclose(self._refresh_hz, 90, abs_tol=0.01):
            return Health(False, now, f"requested 90 Hz; runtime reports {self._refresh_hz:g} Hz")
        return Health(True, sample.header.received_monotonic_ns,
                      "valid frame received; source freshness requires clock mapping")

    def close(self) -> None:
        self._running = False
        if self._process is not None:
            self._process.terminate()
            try:
                self._process.wait(timeout=2)
            except subprocess.TimeoutExpired:
                self._process.kill()
                self._process.wait(timeout=2)
        if self._thread is not None:
            self._thread.join(timeout=2)
        if self._process is not None and self._process.stdout is not None:
            self._process.stdout.close()
        self._process = self._thread = None
        if self._owns_app and self.serial is not None:
            try:
                self._adb("-s", self.serial, "shell", "am", "force-stop", PACKAGE)
            except (OSError, subprocess.SubprocessError) as exc:
                LOGGER.warning("Could not stop owned Quest app: %s", exc)
            self._owns_app = False
        if self._power_override_requested and self.serial is not None:
            try:
                self._adb("-s", self.serial, "shell", "am", "broadcast", "-a", POWER_RESTORE_ACTION)
            except (OSError, subprocess.SubprocessError) as exc:
                LOGGER.warning("Could not restore Quest sleep detection: %s; reconnect USB and run "
                               "adb -s %s shell am broadcast -a %s", exc, self.serial, POWER_RESTORE_ACTION)
            else:
                self._power_override_requested = False
