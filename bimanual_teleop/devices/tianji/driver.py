"""One owner for Tianji's two arms, using the project's small native bridge.

The watchdog is a host-side stop request, not a controller safety guarantee.
"""

from __future__ import annotations

import ctypes as ct
from dataclasses import asdict, dataclass, replace
import errno
import logging
import math
from pathlib import Path
import threading
import time
from typing import Mapping
import uuid

from bimanual_teleop.devices.interfaces import RealtimeObserver
from bimanual_teleop.types import (
    CommandEvent, CommandStatus, ControlProfile, DeviceCommand, Event, Health,
    JointState, Pose, Sample, SampleHeader, SampleRef, Side, Submission,
)
from bimanual_teleop.devices.tianji.model import MotionProfile

LOGGER = logging.getLogger(__name__)
SIDES: tuple[Side, Side] = ("left", "right")
SOURCE_MODULUS = 1_000_000


class _Feedback(ct.Structure):
    _fields_ = (
        [("received_ns", ct.c_uint64), ("packet_index", ct.c_uint64)]
        + [(name, ct.c_int32 * 2) for name in (
            "sequence", "input_sequence", "state", "commanded_state", "error", "impedance_type", "low_speed")]
        + [(name, ct.c_double * 14) for name in ("q", "dq", "current", "torque", "external_torque")]
        + [("external_wrench", ct.c_double * 12)]
        + [(name, ct.c_double * 14) for name in ("target", "cart_k", "cart_d")]
        + [("tool_pose", ct.c_double * 12), ("tool_dynamics", ct.c_double * 20)]
        + [(name, ct.c_int32 * 2) for name in ("velocity_ratio", "acceleration_ratio", "force_type")]
        + [("impedance_rotation", ct.c_double * 14)]
        + [(name, ct.c_int32 * 2) for name in ("pvt_run_id", "pvt_run_state")]
        + [("identification_tag", ct.c_double * 2)]
    )


class _Send(ct.Structure):
    _fields_ = [
        ("token", ct.c_uint64), ("sent_ns", ct.c_uint64), ("packet_index", ct.c_uint64),
        ("result", ct.c_int64), ("error_number", ct.c_int32), ("attempted", ct.c_int32),
        ("size", ct.c_uint32), ("payload", ct.c_uint8 * 1500),
    ]


class _Stats(ct.Structure):
    _fields_ = [(name, ct.c_uint64) for name in (
        "received", "feedback_dropped", "sends", "send_dropped", "first_feedback_lost",
        "last_feedback_lost", "first_send_lost", "last_send_lost")]


class _Profile(ct.Structure):
    _fields_ = [
        ("k", ct.c_double * 7), ("d", ct.c_double * 7), ("tool_pose", ct.c_double * 6),
        ("tool_dynamics", ct.c_double * 10), ("velocity_ratio", ct.c_int32),
        ("acceleration_ratio", ct.c_int32),
    ]


class _NativeBridge:
    def __init__(self, path: Path):
        self.lib = ct.CDLL(str(path))
        self.lib.tj_abi_version.argtypes, self.lib.tj_abi_version.restype = [], ct.c_int
        if self.lib.tj_abi_version() != 2:
            raise RuntimeError("Unsupported Tianji bridge ABI; rebuild tianji_bridge")
        specs = {
            "tj_last_error": [], "tj_open": [ct.c_char_p], "tj_close": [],
            "tj_versions": [ct.POINTER(ct.c_int64), ct.POINTER(ct.c_int64)],
            "tj_download_config": [ct.c_char_p],
            "tj_get_int": [ct.c_char_p, ct.POINTER(ct.c_int64)],
            "tj_poll_feedback": [ct.POINTER(_Feedback)], "tj_poll_send": [ct.POINTER(_Send)],
            "tj_stats": [ct.POINTER(_Stats)],
            "tj_configure": [ct.c_uint32, ct.POINTER(_Profile), ct.c_uint64],
            "tj_engage": [ct.c_uint32, ct.POINTER(ct.c_double), ct.c_uint64, ct.c_uint64],
            "tj_confirm_cartesian": [ct.c_uint32],
            "tj_reset_emergency": [ct.c_int32, ct.c_uint64, ct.c_uint64],
            "tj_submit": [ct.c_uint32, ct.POINTER(ct.c_double), ct.c_uint64, ct.c_uint64],
            "tj_hold": [ct.c_uint32, ct.c_uint64],
            "tj_position_mode": [ct.c_int32, ct.POINTER(ct.c_double), ct.c_uint64, ct.c_uint64],
            "tj_move_joints": [ct.c_int32, ct.POINTER(ct.c_double), ct.c_int32, ct.c_int32,
                               ct.c_uint64, ct.c_uint64],
        }
        for name, args in specs.items():
            function = getattr(self.lib, name)
            function.argtypes, function.restype = args, ct.c_int
        self.lib.tj_last_error.restype = ct.c_char_p

    def call(self, name: str, *args: object) -> int:
        result = getattr(self.lib, name)(*args)
        if result < 0:
            detail = self.lib.tj_last_error()
            raise RuntimeError(f"{name}: {detail.decode() if detail else result}")
        return result

    def poll_feedback(self) -> _Feedback | None:
        item = _Feedback()
        return item if self.call("tj_poll_feedback", ct.byref(item)) else None

    def poll_send(self) -> _Send | None:
        item = _Send()
        return item if self.call("tj_poll_send", ct.byref(item)) else None

    def stats(self) -> _Stats:
        value = _Stats()
        self.call("tj_stats", ct.byref(value))
        return value

    def versions(self) -> tuple[int, int]:
        sdk, controller = ct.c_int64(), ct.c_int64()
        self.call("tj_versions", ct.byref(sdk), ct.byref(controller))
        return sdk.value, controller.value

    def get_int(self, name: str) -> int:
        value = ct.c_int64()
        self.call("tj_get_int", name.encode(), ct.byref(value))
        return value.value


@dataclass(frozen=True)
class TianjiArmState:
    joints: JointState
    source_sequence: int
    input_sequence: int
    state: int
    commanded_state: int
    error: int
    impedance_type: int
    low_speed: int
    controller_target_rad: tuple[float | None, ...]
    native_current_permille: tuple[float | None, ...]


@dataclass(frozen=True)
class TianjiFrame:
    packet_index: int
    arms: Mapping[Side, TianjiArmState]


@dataclass(frozen=True)
class TianjiJointCommand:
    targets: Mapping[Side, tuple[float, ...]]
    cartesian_targets: Mapping[Side, Pose]


def decode_feedback(packet: _Feedback, epoch: str) -> Sample[TianjiFrame]:
    arms = {}
    for index, side in enumerate(SIDES):
        def values(name: str) -> tuple[float | None, ...]:
            return tuple(x if math.isfinite(x) else None for x in getattr(packet, name)[index * 7:(index + 1) * 7])

        radians = lambda name: tuple(math.radians(x) if x is not None else None for x in values(name))
        joints = JointState(
            tuple(f"{side}_joint_{j + 1}" for j in range(7)), radians("q"), radians("dq"),
            measured_torque_nm=values("torque"), estimated_external_torque_nm=values("external_torque"),
        )
        arms[side] = TianjiArmState(
            joints, packet.sequence[index], packet.input_sequence[index], packet.state[index],
            packet.commanded_state[index], packet.error[index], packet.impedance_type[index],
            packet.low_speed[index], radians("target"), values("current"),
        )
    valid = all(x is not None for arm in arms.values() for channel in (
        arm.joints.position_rad, arm.joints.velocity_rad_s, arm.joints.measured_torque_nm,
        arm.joints.estimated_external_torque_nm, arm.controller_target_rad, arm.native_current_permille,
    ) for x in channel)
    return Sample(SampleHeader(
        SampleRef("tianji.feedback", epoch, packet.packet_index), packet.received_ns, valid,
    ), TianjiFrame(packet.packet_index, arms))


class TianjiDriver:
    """Live feedback start; apply a profile before explicit engage.

    Observer methods must be nonblocking. Freshness means a side's source counter
    advanced recently at this host; no device timestamp or clock mapping exists.
    """

    def __init__(self, controller_ip: str, library_path: str | Path | None = None,
                 *, model_path: str | Path | None = None, watchdog_s: float = 0.05,
                 engagement_timeout_s: float = 1.0):
        from .model import DEFAULT_LIBRARY
        if not math.isfinite(watchdog_s) or watchdog_s <= 0:
            raise ValueError("watchdog_s must be positive and finite")
        if not math.isfinite(engagement_timeout_s) or engagement_timeout_s <= 0:
            raise ValueError("engagement_timeout_s must be positive and finite")
        self.controller_ip = controller_ip
        self.library_path = Path(library_path or DEFAULT_LIBRARY).resolve()
        self.model_path = model_path
        self.watchdog_ns = int(watchdog_s * 1e9)
        self.engagement_timeout_ns = int(engagement_timeout_s * 1e9)
        self.profile: MotionProfile | None = None
        self.engaged = False
        self.engagement_sample: Sample[TianjiFrame] | None = None
        self._engaged_ns = 0
        self._native: _NativeBridge | None = None
        self._sink: RealtimeObserver | None = None
        self._lock = threading.RLock()
        self._stop = threading.Event()
        self._receiver: threading.Thread | None = None
        self._watchdog: threading.Thread | None = None
        self._latest: Sample[TianjiFrame] | None = None
        self._packet: _Feedback | None = None
        self._advanced: dict[Side, int] = {}
        self._sequences: dict[Side, int] = {}
        self._reordered_sides: set[Side] = set()
        self._reported_config: dict[Side, object] = {}
        self._problem: str | None = None
        self._observer_error: str | None = None
        self._hold_reason: str | None = None
        self._hold_returned_ns: int | None = None
        self._held_feedback = False
        self._mode_confirmed = False
        self._control_mode = "cartesian"
        self._native_move_side: Side | None = None
        self._hold_sides: tuple[Side, ...] = ()
        self._motion_fault: str | None = None
        self._deadline_ns = 0
        self._last_target_ns = 0
        self._last_targets: Mapping[Side, tuple[float, ...]] = {}
        self._token = 0
        self._commands: dict[int, str] = {}
        self._send_results: dict[int, int | str] = {}
        self._dropped = (0, 0)
        self._epoch = uuid.uuid4().hex
        self._metadata: dict[str, object] = {
            "controller_ip": controller_ip, "library_path": str(self.library_path),
            "host_watchdog_s": watchdog_s, "source_clock": None,
            "engagement_timeout_s": engagement_timeout_s,
            "feedback_clock": "host CLOCK_MONOTONIC at each native UDP receive",
            "freshness": "host elapsed since each arm's source counter advanced; not source age",
            "current_unit": "native permille; not amperes",
            "torque_unit": "Nm; sensor torque and vendor disturbance estimate are separate",
            "controller_target": "controller-reported m_FB_Joint_Cmd, converted from degrees to radians",
            "tool_frame": "flange identity; each arm has its own base frame",
            "send_receipt": "local sendto result; not controller execution confirmation",
        }

    @property
    def metadata(self) -> Mapping[str, object]:
        return dict(self._metadata, epoch=self._epoch, reported_config=dict(self._reported_config),
                    problem=self._problem)

    def start(self, sink: RealtimeObserver | None = None) -> None:
        if self._native is not None:
            raise RuntimeError("Tianji driver is already started")
        if self._stop.is_set():
            raise RuntimeError("Create a new driver after close")
        self._sink = sink
        native = _NativeBridge(self.library_path)
        opened = False
        try:
            native.call("tj_open", self.controller_ip.encode())
            opened = True
            self._native = native
            sdk, controller = native.versions()
            self._metadata.update(sdk_version=sdk, controller_version=controller)
            self._receiver = threading.Thread(target=self._receive, name="tianji-receive", daemon=True)
            self._watchdog = threading.Thread(target=self._watch, name="tianji-watchdog", daemon=True)
            self._receiver.start()
            self._watchdog.start()
            self._event("tianji.started", self.metadata)
            if self._problem:
                raise RuntimeError(self._problem)
        except Exception:
            self._stop.set()
            if self._receiver and self._receiver.is_alive():
                self._receiver.join(timeout=2)
            if opened:
                native.call("tj_close")
            self._native = None
            raise

    def _connected(self) -> _NativeBridge:
        if self._native is None or self._stop.is_set():
            raise RuntimeError("Tianji driver is not running")
        return self._native

    def _publish(self, sample: Sample[object]) -> None:
        if self._sink is None or self._observer_error is not None:
            return
        try:
            if self._sink.try_publish(sample):
                return
            self._observer_failure()
        except Exception as error:
            self._observer_failure(error)

    def _event(self, kind: str, details: Mapping[str, object], stamp: int | None = None) -> None:
        self._emit_event(Event(kind, time.monotonic_ns() if stamp is None else stamp, "tianji", details))

    def _emit_event(self, event: Event | CommandEvent) -> None:
        if self._sink is None or self._observer_error is not None:
            return
        try:
            if self._sink.try_event(event):
                return
            self._observer_failure()
        except Exception as error:
            self._observer_failure(error)

    def _observer_failure(self, error: Exception | None = None) -> None:
        if self._observer_error is not None:
            return
        detail = (f"Tianji realtime observer failed: {error}" if error else
                  "Tianji realtime observer rejected live data")
        self._observer_error = self._problem = detail
        self._metadata["observer_error"] = detail
        LOGGER.error(detail)

    def get_latest(self) -> Sample[TianjiFrame] | None:
        return self._latest

    def _feedback_problem(self, sides: tuple[Side, ...], now: int, *, idle: bool = False) -> str | None:
        sample = self._latest
        if sample is None:
            return "No Tianji feedback"
        for side in sides:
            arm = sample.payload.arms[side]
            if side in self._reordered_sides:
                return f"{side} source counter reset/reordered; await a subsequent advancing sample"
            if now - self._advanced.get(side, 0) >= self.watchdog_ns:
                return f"{side} source counter has not advanced within the host watchdog interval"
            if arm.error:
                return f"{side} controller error {arm.error}"
            if any(q is None for channel in (arm.joints.position_rad, arm.joints.velocity_rad_s) for q in channel):
                return f"{side} joint position/velocity contains invalid values"
            if self.engaged and self._mode_confirmed and not self._in_control_mode(arm):
                return f"{side} is not reporting the engaged {self._control_mode} mode"
        return None

    def _in_control_mode(self, arm: TianjiArmState) -> bool:
        return (arm.state == 1 if self._control_mode == "position"
                else arm.state == 3 and arm.impedance_type == 2)

    def health(self, *, sides: tuple[Side, ...] | None = None) -> Health:
        now = time.monotonic_ns()
        sides = (self.profile.active_arms if self.profile else SIDES) if sides is None else tuple(sides)
        if not sides or len(set(sides)) != len(sides) or any(side not in SIDES for side in sides):
            raise ValueError("Tianji health sides must be left, right, or both")
        problem = self._problem or (self._motion_fault if self.engaged else None) or self._feedback_problem(sides, now)
        if self._native is None or self._stop.is_set():
            problem = "Tianji driver is closed"
        elif self._hold_reason:
            problem = f"Hold request pending: {self._hold_reason}"
        return Health(problem is None, now, problem)

    def _on_feedback(self, packet: _Feedback) -> None:
        sample = decode_feedback(packet, self._epoch)
        for side, arm in sample.payload.arms.items():
            previous = self._sequences.get(side)
            delta = None if previous is None else (arm.source_sequence - previous) % SOURCE_MODULUS
            reset = delta is not None and delta >= SOURCE_MODULUS // 2
            if reset:
                if self.profile is None or side in self.profile.active_arms:
                    self._motion_fault = f"{side} source sequence reset/reordered; explicit re-engagement required"
                self._reordered_sides.add(side)
            elif previous is None or previous != arm.source_sequence:
                self._advanced[side] = packet.received_ns
                self._reordered_sides.discard(side)
            if previous is not None:
                if delta > 1:
                    self._event("tianji.source_gap" if delta < SOURCE_MODULUS // 2 else "tianji.source_reset",
                                {"side": side, "previous": previous, "current": arm.source_sequence,
                                 "missing": delta - 1 if delta < SOURCE_MODULUS // 2 else None,
                                 "packet_index": packet.packet_index}, packet.received_ns)
            self._sequences[side] = arm.source_sequence
            config = self._packet_config(packet, SIDES.index(side))
            if self._reported_config.get(side) != config:
                self._reported_config[side] = config
                self._event("tianji.reported_config", {"side": side, "configuration": config,
                            "packet_index": packet.packet_index}, packet.received_ns)
        self._packet, self._latest = packet, sample
        stable_low_speed = self.profile is not None and all(
            arm.low_speed == 1 and (arm.state == 0 or self._in_control_mode(arm)) and
            all(q is not None for q in arm.joints.velocity_rad_s)
            for side, arm in sample.payload.arms.items() if side in (self._hold_sides or self.profile.active_arms))
        if self._held_feedback and not stable_low_speed:
            self._held_feedback = False
        if self._hold_returned_ns is not None and not self._held_feedback and self.profile:
            if stable_low_speed and all(self._advanced.get(side, 0) > self._hold_returned_ns
                                        for side in (self._hold_sides or self.profile.active_arms)):
                self._held_feedback = True
                self._event("tianji.hold_low_speed_observed", {
                    "packet_index": packet.packet_index,
                    "joint_velocity_rad_s": {side: sample.payload.arms[side].joints.velocity_rad_s
                                             for side in self.profile.active_arms},
                    "meaning": "fresh controller low-speed flag in idle/owned control mode; not an independent physical-stop measurement",
                }, packet.received_ns)
        if not sample.header.valid:
            self._event("tianji.invalid_feedback", {"packet_index": packet.packet_index}, packet.received_ns)
        self._publish(sample)

    @staticmethod
    def _packet_config(packet: _Feedback, index: int) -> dict[str, object]:
        def array(name, width):
            return tuple(x if math.isfinite(x) else None
                         for x in getattr(packet, name)[index * width:(index + 1) * width])
        return {"stiffness_native": array("cart_k", 7), "damping_native": array("cart_d", 7),
                "tool_pose_mm_deg": array("tool_pose", 6), "tool_dynamics_native": array("tool_dynamics", 10),
                "velocity_ratio_percent": packet.velocity_ratio[index],
                "acceleration_ratio_percent": packet.acceleration_ratio[index],
                "force_type": packet.force_type[index], "impedance_rotation_native": array("impedance_rotation", 7)}

    def _on_send(self, packet: _Send) -> None:
        sent = bool(packet.attempted) and packet.result == packet.size
        self._send_results[packet.token] = (packet.sent_ns if sent else
            f"Tianji UDP send failed (token {packet.token}, errno {packet.error_number})")
        command_id = self._commands.get(packet.token)
        if command_id is not None:
            self._emit_event(CommandEvent("tianji", command_id, CommandStatus.SENT if sent else CommandStatus.SEND_FAILED,
                                          packet.sent_ns, "sendto completed" if sent else
                                          f"attempted={bool(packet.attempted)}, errno={packet.error_number}"))
        if not sent:
            expired = not packet.attempted and packet.error_number == errno.ETIMEDOUT
            kind = "tianji.command_expired" if expired else "tianji.send_failed"
            self._event(kind, {"native_token": packet.token, "attempted": bool(packet.attempted),
                              "result": packet.result, "errno": packet.error_number}, packet.sent_ns)
            if expired:
                self._motion_fault = "Target expired before send; explicit re-engagement required"
            else:
                self._problem = f"Tianji UDP send failed (token {packet.token}, errno {packet.error_number})"

    def _check_stats(self, stats: _Stats) -> None:
        for index, stream in enumerate(("feedback", "send")):
            count = getattr(stats, f"{stream}_dropped")
            if count != self._dropped[index]:
                self._metadata["queue_overflow"] = f"Native {stream} queue overflow"
                if self._dropped[index] == 0:
                    LOGGER.warning("Tianji native %s queue overflow: %s lost", stream, count)
                self._event("tianji.native_queue_overflow", {
                    "stream": stream, "new_losses": count - self._dropped[index], "total_losses": count,
                    "first_lost_packet_index": getattr(stats, f"first_{stream}_lost"),
                    "last_lost_packet_index": getattr(stats, f"last_{stream}_lost"),
                    "bounds_are_not_contiguous_loss_interval": True,
                })
        self._dropped = (stats.feedback_dropped, stats.send_dropped)

    def _drain(self) -> int:
        native = self._native
        if native is None:
            return 0
        consumed = 0
        for _ in range(256):
            packet = native.poll_feedback()
            if packet is None:
                break
            self._on_feedback(packet)
            consumed += 1
        for _ in range(256):
            packet = native.poll_send()
            if packet is None:
                break
            self._on_send(packet)
            consumed += 1
        self._check_stats(native.stats())
        return consumed

    def _receive(self) -> None:
        try:
            while not self._stop.is_set():
                self._drain()
                self._stop.wait(0.002)
        except Exception as error:
            self._problem = f"Tianji receiver failed: {error}"
            self._event("tianji.receiver_failed", {"error": str(error)})

    def _watch(self) -> None:
        while not self._stop.wait(min(0.005, self.watchdog_ns / 4e9)):
            now = time.monotonic_ns()
            if self.engaged and self.profile:
                sides = (self._native_move_side,) if self._native_move_side else self.profile.active_arms
                reason = self._problem or self._motion_fault or self._feedback_problem(sides, now)
                if self._native_move_side is None and now >= self._deadline_ns:
                    reason = reason or "Accepted target expired or no target arrived within host watchdog interval"
                if reason:
                    self.request_hold(reason)
            elif self._hold_reason:
                self.request_hold(self._hold_reason)

    def download_config(self, path: str | Path) -> None:
        """Download persisted robot.ini; this does not read unsaved live settings."""
        self._connected().call("tj_download_config", str(Path(path).resolve()).encode())

    def _next_token(self, command_id: str) -> int:
        self._token += 1
        self._commands[self._token] = command_id
        # Keep bounded command-to-send correlation in memory.
        if len(self._commands) > 2048:
            oldest = min(self._commands)
            del self._commands[oldest]
            self._send_results.pop(oldest, None)
        return self._token

    def _wait_sent(self, token: int) -> int:
        deadline = time.monotonic_ns()+self.watchdog_ns
        while True:
            result = self._send_results.get(token)
            if isinstance(result, str):
                raise RuntimeError(result)
            if result is not None:
                return result
            if self._problem or self._stop.is_set():
                raise RuntimeError(self._problem or "Tianji driver closed before send")
            if time.monotonic_ns() >= deadline:
                raise RuntimeError(f"Tianji send result missing for token {token}")
            self._stop.wait(.001)

    @staticmethod
    def _mask(sides: tuple[Side, ...]) -> int:
        return sum(1 << SIDES.index(side) for side in sides)

    def _require_feedback(self, sides: tuple[Side, ...], *, idle: bool = False) -> None:
        reason = self._problem or self._hold_reason or (self._motion_fault if self.engaged else None) or self._feedback_problem(sides, time.monotonic_ns(), idle=idle)
        if reason:
            raise RuntimeError(reason)

    def reset_released_emergency(self, *, physical_release_confirmed: bool = False) -> dict:
        """Explicitly clear only error 13 after operator confirmation of release.

        RESET0/RESET1 is sent once as needed. Controller feedback confirms
        disabled/error-free state; this method does not enable the arms.
        """
        if physical_release_confirmed is not True:
            raise ValueError("Physical emergency release must be explicitly confirmed")
        with self._lock:
            native = self._connected()
            if self.engaged or self.profile is not None or self._hold_reason:
                raise RuntimeError("Emergency reset requires a new capture-only driver")

            def snapshot():
                now, sample = time.monotonic_ns(), self._latest
                if self._problem or sample is None:
                    raise RuntimeError(self._problem or "No feedback for emergency reset")
                for side, arm in sample.payload.arms.items():
                    if side in self._reordered_sides or now-self._advanced.get(side, 0) >= self.watchdog_ns:
                        raise RuntimeError(f"{side} emergency reset feedback is stale/reordered")
                    if (arm.state, arm.error) not in ((0, 0), (100, 13)):
                        raise RuntimeError(f"{side} must be disabled or have only emergency fault 13")
                    if any(q is None for channel in (arm.joints.position_rad, arm.joints.velocity_rad_s)
                           for q in channel):
                        raise RuntimeError(f"{side} emergency reset feedback contains invalid joints")
                return sample

            initial = snapshot()
            targets = [s for s in SIDES if initial.payload.arms[s].error == 13]
            result = {"physical_release_confirmed": True, "requested_arms": targets,
                      "initial_feedback": asdict(initial), "confirmed_disabled": False}
            self._event("tianji.emergency_reset_requested", result)
            for side in targets:
                sample = snapshot()
                if sample.payload.arms[side].error == 0:
                    continue
                token = self._next_token(f"reset-emergency-{side}-{uuid.uuid4().hex}")
                now = time.monotonic_ns()
                native.call("tj_reset_emergency", SIDES.index(side), token, now+self.watchdog_ns)
                self._event("tianji.emergency_reset_sdk_returned", {"side": side, "native_token": token,
                            "controller_state_confirmed": False})
            deadline = time.monotonic()+2.
            while time.monotonic() < deadline:
                sample = snapshot()
                if all(sample.payload.arms[s].state == 0 and sample.payload.arms[s].error == 0 for s in SIDES):
                    result.update(confirmed_disabled=True, final_feedback=asdict(sample))
                    self._event("tianji.emergency_reset_confirmed", result)
                    return result
                self._stop.wait(.005)
            raise RuntimeError("Emergency reset did not produce disabled/error-free feedback; no retry")

    def configure(self, profile: ControlProfile) -> None:
        parsed = MotionProfile.from_control_profile(profile, self.model_path)
        with self._lock:
            native = self._connected()
            profiles = (_Profile * 2)()
            for side in parsed.active_arms:
                arm = parsed.arms[side]
                p = profiles[SIDES.index(side)]
                p.k[:] = (*arm.stiffness, arm.nullspace_stiffness)
                p.d[:] = (*arm.damping, arm.nullspace_damping)
                p.tool_dynamics[:] = arm.tool_dyn10
                p.velocity_ratio, p.acceleration_ratio = arm.velocity_ratio, arm.acceleration_ratio
            token = self._next_token(f"configure-{uuid.uuid4().hex}")
            self._event("tianji.configure_requested", {"native_token": token, "profile": asdict(profile)})
            native.call("tj_configure", self._mask(parsed.active_arms), profiles, token)
            self._wait_sent(token)
            self.profile = parsed
            self._metadata["control_profile"] = asdict(profile)
            self._event("tianji.configure_sdk_returned", {"native_token": token, "profile_id": profile.profile_id})

    def engage(self) -> None:
        self._engage("cartesian")

    def engage_position(self) -> None:
        """Seed position mode from measured joints."""
        self._engage("position")

    def adopt_stationary_position(self, profile: ControlProfile, expected_rad: tuple[float, ...]) -> None:
        """Compatibility entry point; apply the profile before engagement."""
        self.configure(profile)

    def adopt_stationary_control(self, profile: ControlProfile) -> None:
        """Compatibility entry point; apply the profile before engagement."""
        self.configure(profile)

    def move_joints(self, side: Side, target_rad: tuple[float, ...]) -> Sample[TianjiFrame]:
        """Run one controller-planned joint move and wait for its final target feedback."""
        target = tuple(target_rad)
        if len(target) != 7 or any(q is None or not math.isfinite(q) for q in target):
            raise ValueError("Joint move requires seven finite joints")
        with self._lock:
            native = self._connected()
            if self.profile is None or side not in self.profile.active_arms:
                raise RuntimeError("Joint move requires a configured arm")
            if self.engaged:
                raise RuntimeError("Tianji is already engaged")
            self._require_feedback((side,))
            angles = tuple(math.degrees(q) for q in target)
            encoded_target = tuple(ct.c_float(q).value for q in angles)
            arm_profile = self.profile.arms[side]
            self._native_move_side, self._control_mode = side, "position"
            self.engaged, self._mode_confirmed = True, False
            self._motion_fault = None
            self._hold_returned_ns, self._held_feedback = None, False
            try:
                index = SIDES.index(side)
                if self._latest.payload.arms[side].state != 1:
                    # The controller can replace a target while entering position
                    # mode. Confirm the mode first, as in the vendor position demo.
                    seed = tuple(math.degrees(q) for q in self._latest.payload.arms[side].joints.position_rad)
                    token = self._next_token(f"position-mode-{side}-{uuid.uuid4().hex}")
                    now = time.monotonic_ns()
                    self._event("tianji.position_mode_requested", {"side": side, "seed_deg": seed,
                                                                 "native_token": token})
                    native.call("tj_move_joints", index, (ct.c_double * 7)(*seed),
                                arm_profile.velocity_ratio, arm_profile.acceleration_ratio,
                                token, now+self.watchdog_ns)
                    mode_sent_ns = self._wait_sent(token)
                    deadline = now+self.engagement_timeout_ns
                    while True:
                        self._require_feedback((side,))
                        if (self._advanced.get(side, 0) > mode_sent_ns
                                and self._latest.payload.arms[side].state == 1):
                            break
                        if time.monotonic_ns() >= deadline:
                            raise RuntimeError(f"{side} position mode was not reported before the startup deadline")
                        self._stop.wait(.005)
                self._mode_confirmed = True
                token = self._next_token(f"joint-move-{side}-{uuid.uuid4().hex}")
                now = time.monotonic_ns()
                self._event("tianji.joint_move_requested", {"side": side, "target_rad": target,
                            "native_token": token, "velocity_ratio": arm_profile.velocity_ratio,
                            "acceleration_ratio": arm_profile.acceleration_ratio})
                command = (ct.c_double * 14)()
                command[index*7:index*7+7] = angles
                native.call("tj_submit", 1 << index, command, token, now+self.watchdog_ns)
                sent_ns = self._wait_sent(token)
                while True:
                    self._require_feedback((side,))
                    sample = self._latest
                    arm = sample.payload.arms[side]
                    if any(q is None for q in arm.controller_target_rad):
                        raise RuntimeError(f"{side} controller target contains invalid joints")
                    reported_target = tuple(ct.c_float(math.degrees(q)).value for q in arm.controller_target_rad)
                    if (self._advanced.get(side, 0) > sent_ns
                            and arm.state == 1 and arm.low_speed == 1
                            and reported_target == encoded_target):
                        self.engaged = False
                        self._event("tianji.joint_move_completed", {"side": side, "feedback": asdict(sample)})
                        return sample
                    self._stop.wait(.005)
            except BaseException:
                self.request_hold("joint move interrupted")
                raise
            finally:
                self._native_move_side = None

    def _engage(self, mode: str) -> None:
        with self._lock:
            self._connected()
            if self.profile is None:
                raise RuntimeError("A configured profile is required before engage")
            if self.engaged:
                raise RuntimeError("Tianji is already engaged")
            self._require_feedback(self.profile.active_arms)
            sample = self._latest
            targets = {side: sample.payload.arms[side].joints.position_rad for side in self.profile.active_arms}
            now = time.monotonic_ns()
            command = DeviceCommand("tianji", f"engage-{uuid.uuid4().hex}", TianjiJointCommand(targets, {}),
                                    (sample.header.ref,), now, now + self.engagement_timeout_ns, self.profile.profile_id)
            self._motion_fault = None
            self._mode_confirmed = False
            self._control_mode = mode
            result = self._send_command(command, "tj_position_mode" if mode == "position" else "tj_engage")
            if not result.accepted:
                raise RuntimeError(result.reason)
            self.engaged = True
            self._engaged_ns = now
            self.engagement_sample = sample
            self._hold_returned_ns, self._held_feedback = None, False
            self._hold_sides = ()
            self._mode_confirmed = False
            while time.monotonic_ns() < self._deadline_ns:
                # engage owns the command lock while waiting, so the watchdog
                # thread cannot issue hold here. Check feedback in this loop too.
                reason = self._problem or self._motion_fault or self._feedback_problem(
                    self.profile.active_arms, time.monotonic_ns())
                if reason:
                    self.request_hold(reason)
                    raise RuntimeError(f"Cannot confirm {mode} engagement: {reason}")
                latest = self._latest
                if all(self._advanced.get(side, 0) >= now and self._in_control_mode(latest.payload.arms[side])
                       for side in self.profile.active_arms):
                    if mode == "cartesian":
                        try:
                            self._native.call("tj_confirm_cartesian", self._mask(self.profile.active_arms))
                        except RuntimeError:
                            self.request_hold("Native Cartesian mode confirmation failed")
                            raise
                    # Only the measured seed has a startup grace period. Once
                    # the mode is ready, a fresh target is due within watchdog_s.
                    confirmed_ns = time.monotonic_ns()
                    self._deadline_ns = min(self._deadline_ns, confirmed_ns + self.watchdog_ns)
                    self._mode_confirmed = True
                    self._event("tianji.engagement_mode_reported", {"packet_index": latest.payload.packet_index,
                                                                 "command_id": command.command_id,
                                                                 "control_mode": mode,
                                                                 "elapsed_ms": (confirmed_ns - now) / 1e6,
                                                                 "first_target_deadline_ns": self._deadline_ns})
                    return
                self._stop.wait(0.002)
            self.request_hold("Engagement mode was not reported before the startup deadline")
            raise RuntimeError(f"Controller did not confirm {mode} engagement before startup timeout")

    def submit(self, command: DeviceCommand[TianjiJointCommand]) -> Submission:
        with self._lock:
            try:
                self._connected()
                if not self.engaged or self.profile is None:
                    raise RuntimeError("Tianji is not engaged")
                if not self._mode_confirmed:
                    raise RuntimeError("Engagement mode has not been reported")
                self._require_feedback(self.profile.active_arms)
                return self._send_command(command, "tj_submit")
            except (ValueError, RuntimeError) as error:
                self._emit_event(CommandEvent("tianji", command.command_id, CommandStatus.REJECTED,
                                              time.monotonic_ns(), str(error)))
                if self.engaged:
                    self.request_hold(str(error))
                return Submission(command.command_id, False, str(error))

    def _send_command(self, command: DeviceCommand[TianjiJointCommand], operation: str) -> Submission:
        now, profile = time.monotonic_ns(), self.profile
        if command.device_id != "tianji" or command.control_profile_id != profile.profile_id:
            raise ValueError("Device or control profile does not match Tianji")
        if command.expires_monotonic_ns <= now or command.created_monotonic_ns > now:
            raise ValueError("Target expired or has a future creation time")
        if operation == "tj_submit" and now >= self._deadline_ns:
            raise ValueError("Previous target watchdog expired; re-engage before submitting")
        if set(command.payload.targets) != set(profile.active_arms):
            raise ValueError("Targets must cover exactly the configured active arms")
        command = replace(command, source_refs=tuple(command.source_refs), payload=TianjiJointCommand(
            {side: tuple(positions) for side, positions in command.payload.targets.items()},
            dict(command.payload.cartesian_targets)))
        q = (ct.c_double * 14)()
        for side, positions in command.payload.targets.items():
            if len(positions) != 7 or any(value is None or not math.isfinite(value) for value in positions):
                raise ValueError(f"{side} target must contain seven finite joints")
            q[SIDES.index(side) * 7:(SIDES.index(side) + 1) * 7] = tuple(math.degrees(value) for value in positions)
        token = self._next_token(command.command_id)
        expires = min(command.expires_monotonic_ns, now + self.watchdog_ns)
        if self._problem:
            raise RuntimeError(self._problem)
        if operation == "tj_position_mode":
            for offset, side in enumerate(profile.active_arms):
                if offset:
                    token = self._next_token(command.command_id)
                    expires = min(command.expires_monotonic_ns, time.monotonic_ns()+self.watchdog_ns)
                index = SIDES.index(side)
                arm = profile.arms[side]
                self._native.call("tj_move_joints", index, (ct.c_double * 7)(*q[index*7:index*7+7]),
                                  arm.velocity_ratio, arm.acceleration_ratio, token, expires)
                self._wait_sent(token)
        else:
            self._native.call(operation, self._mask(profile.active_arms), q, token, expires)
        # A seed target may hold still while the servo enables, but must still
        # reach sendto within watchdog_s. Runtime targets keep the shorter limit.
        self._deadline_ns = (min(command.expires_monotonic_ns, now + self.engagement_timeout_ns)
                             if operation in ("tj_engage", "tj_position_mode") else expires)
        self._last_targets, self._last_target_ns = command.payload.targets, now
        self._emit_event(CommandEvent("tianji", command.command_id, CommandStatus.ACCEPTED, time.monotonic_ns()))
        return Submission(command.command_id, True)

    def request_hold(self, reason: str) -> None:
        with self._lock:
            if not self.engaged and not self._hold_reason:
                return
            self.engaged = False
            self._mode_confirmed = False
            if self._hold_reason is None:
                self._hold_sides = (self._native_move_side,) if self._native_move_side else self.profile.active_arms
                self._event("tianji.hold_requested", {"reason": reason, "active_arms": self._hold_sides})
            sides = self._hold_sides
            self._hold_reason = reason
            token = self._next_token(f"hold-{uuid.uuid4().hex}")
            method = "sdk_stop"
            try:
                self._native.call("tj_hold", self._mask(sides), token)
            except RuntimeError as error:
                # A prior datagram can still be pending; the watchdog retries.
                self._metadata["last_hold_error"] = str(error)
                # A controller may reject RSTA on an already completed position
                # stream. Explicitly replace that target with the measured pose,
                # only in fresh, error-free position mode. No PVT or torque-mode
                # stop is replaced by this fallback. Feedback must still confirm.
                if (self._control_mode != "position" or
                        "returned 1" not in str(error) or self._feedback_problem(sides, time.monotonic_ns()) or
                        any(self._latest.payload.arms[s].state != 1 for s in sides)):
                    return
                sample = self._latest
                try:
                    for side in sides:
                        token = self._next_token(f"position-hold-{side}-{uuid.uuid4().hex}")
                        q = (ct.c_double * 7)(*(math.degrees(v) for v in sample.payload.arms[side].joints.position_rad))
                        arm = self.profile.arms[side]
                        self._native.call("tj_move_joints", SIDES.index(side), q,
                                          arm.velocity_ratio, arm.acceleration_ratio, token,
                                          time.monotonic_ns()+self.watchdog_ns)
                        self._wait_sent(token)
                except (RuntimeError, ValueError) as fallback_error:
                    self._metadata["position_hold_error"] = str(fallback_error)
                    return
                self._event("tianji.position_hold_target_replaced", {"rejected_stop": str(error),
                            "active_arms": sides, "feedback": asdict(sample), "physical_stop_confirmed": False})
                token, method = self._token, "measured_position_target"
            self._hold_reason = None
            self._hold_returned_ns = time.monotonic_ns()
            self._held_feedback = False
            self._event("tianji.hold_sdk_returned", {"native_token": token, "reason": reason,
                                                   "method": method,
                                                   "physical_stop_confirmed": False})

    def close(self) -> None:
        with self._lock:
            if self._native is None:
                return
            self.request_hold("driver close")
        # Let an already pending target expire/drain before one final stop attempt.
        deadline = time.monotonic() + max(0.1, self.watchdog_ns / 1e9 * 2)
        while self._hold_reason and time.monotonic() < deadline:
            time.sleep(0.005)
            if self._hold_reason:
                self.request_hold("driver close")
        self._stop.set()
        for thread in (self._watchdog, self._receiver):
            if thread and thread is not threading.current_thread():
                thread.join(timeout=2)
        try:
            self._native.call("tj_close")
            while self._drain():
                pass
            if self._hold_reason or self._hold_returned_ns is not None and not self._held_feedback:
                self._event("tianji.hold_unconfirmed", {"reason": self._hold_reason or "No fresh post-hold low-speed feedback",
                                                       "error": self._metadata.get("last_hold_error")})
        finally:
            self._native = None
