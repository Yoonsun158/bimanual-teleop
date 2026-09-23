"""Explicit device boundaries for the isolated rail experiment.

Construction is passive. ``open`` connects feedback only; configuration and
engagement are separate, deliberate operations. This module runs no control
loop. The caller must start each hand loop immediately after its engagement,
and the arm loop immediately after arm engagement.
"""

from __future__ import annotations

from copy import deepcopy
from dataclasses import asdict
import fcntl
import hashlib
import json
import math
import os
from pathlib import Path
import threading
import time
from uuid import uuid4

from bimanual_teleop.devices.tianji.config import DEFAULT_CONFIG as TIANJI_CONFIG, load_config as load_tianji
from bimanual_teleop.devices.tianji.driver import TianjiDriver
from bimanual_teleop.devices.tianji.model import TianjiKinematics
from bimanual_teleop.control.arm.cartesian import TianjiCartesianExecutor
from bimanual_teleop.devices.wuji.config import DEFAULT_CONFIG as WUJI_CONFIG, load_config as load_wuji
from bimanual_teleop.devices.wuji.adapter import JOINT_NAMES, WujiDiagnostics, WujiHandDriver
from bimanual_teleop.types import ControlProfile, DeviceCommand, JointTarget, Pose, RobotTarget


SIDES = ("left", "right")
COMMAND_TTL_NS = 50_000_000


def _json_value(value):
    """No NaN/Infinity or SDK objects may escape into experiment records."""
    if isinstance(value, float):
        return value if math.isfinite(value) else None
    if isinstance(value, dict):
        return {str(key): _json_value(item) for key, item in value.items()}
    if isinstance(value, (list, tuple)):
        return [_json_value(item) for item in value]
    if value is None or isinstance(value, (str, int, bool)):
        return value
    raise TypeError(f"snapshot contains unsupported {type(value).__name__}")


def _find_local_controllers(proc_root=Path("/proc")):
    """Conservative local conflict check; cannot exclude remote controllers.

    Loaded control libraries are treated as busy even if their connection is
    unknown. Match command argument basenames, not arbitrary shell text.
    """
    conflicts = []
    commands = {
        "teleop_quest_tianji", "teleop_wuji_hand2", "home_tianji", "jog_tianji",
        "home_wuji_hand2", "clear_tianji_errors", "read_tianji_force",
        "read_tianji_right_force",
    }
    for entry in proc_root.iterdir():
        if not entry.name.isdecimal() or int(entry.name) == os.getpid():
            continue
        try:
            args = (entry / "cmdline").read_bytes().decode(errors="replace").split("\0")
            matches = [arg for arg in args if (
                Path(arg).name.removesuffix(".py") in commands or
                arg.removeprefix("bimanual_teleop.cli.") in commands)]
            maps = (entry / "maps").read_text(errors="replace")
        except (FileNotFoundError, ProcessLookupError):
            continue
        except PermissionError:
            # Foreign-user processes cannot always be inspected. The lock and
            # this scan are local best effort, never controller arbitration.
            continue
        libraries = [name for name in ("libMarvinSDK.so", "/wuji_sdk/") if name in maps]
        if matches or libraries:
            conflicts.append(f"pid={entry.name}: {', '.join(matches + libraries)}")
    return conflicts


class _HardwareLease:
    """Advisory locks shared across worktrees, held until all devices close."""

    def __init__(self, resources):
        self.resources = tuple(sorted(set(resources)))
        self.fds = []

    def acquire(self):
        try:
            for resource in self.resources:
                key = hashlib.sha256(resource.encode()).hexdigest()[:24]
                path = Path("/tmp") / f"astra-rail-hardware-{key}.lock"
                fd = os.open(path, os.O_RDWR | os.O_CREAT | os.O_CLOEXEC | os.O_NOFOLLOW, 0o600)
                try:
                    fcntl.flock(fd, fcntl.LOCK_EX | fcntl.LOCK_NB)
                except (BlockingIOError, OSError):
                    os.close(fd)
                    raise RuntimeError(f"Local hardware lock is busy: {resource}") from None
                self.fds.append(fd)
                os.ftruncate(fd, 0)
                os.write(fd, json.dumps({"pid": os.getpid(), "resource": resource}).encode())
            conflicts = _find_local_controllers()
            if conflicts:
                raise RuntimeError("Other local hardware SDK/controller process: " + "; ".join(conflicts))
        except BaseException:
            self.release()
            raise

    def release(self):
        errors = []
        while self.fds:
            fd = self.fds.pop()
            try:
                os.close(fd)  # Closing releases flock. Never unlink a locked inode.
            except OSError as error:
                errors.append(str(error))
        if errors:
            raise RuntimeError("hardware lock release failed: " + "; ".join(errors))


class HardwareBackend:
    def __init__(self, tianji_config=None, wuji_config=None, sides=SIDES):
        self.sides = tuple(sides)
        if not self.sides or len(set(self.sides)) != len(self.sides) or set(self.sides) - set(SIDES):
            raise ValueError("sides must contain left, right, or both without duplicates")
        self.tianji_settings = load_tianji(tianji_config or TIANJI_CONFIG)
        self.wuji_settings = load_wuji(wuji_config or WUJI_CONFIG)
        parameters = deepcopy(self.tianji_settings["profile"]["parameters"])
        parameters["active_arms"] = list(self.sides)
        parameters["arms"] = {side: parameters["arms"][side] for side in self.sides}
        for arm in parameters["arms"].values():
            arm.update(velocity_ratio=5, acceleration_ratio=5)
        self.arm_profile = ControlProfile("astra-rail-arms-" + "-".join(self.sides),
                                          "cartesian_impedance", parameters)
        self.hand_profile = ControlProfile("astra-rail-hands", "mit",
                                          {"kp": 3., "kd": .05, "current_limit_a": .5})
        resources = ["tianji:" + self.tianji_settings["controller_ip"]]
        resources += ["wuji-hand:" + self.wuji_settings["devices"][side]["hand"] for side in self.sides]
        self._lease = _HardwareLease(resources)
        self.driver = self.kinematics = self.executor = None
        self.hands = {}
        self.errors = []
        self._open_errors = {}
        self._opened = self._closed = self._configured = False
        self._fault_reason = None
        self._state_lock = threading.RLock()
        self._arm_lock = threading.RLock()
        self._hand_locks = {side: threading.RLock() for side in self.sides}

    def _record(self, message):
        with self._state_lock:
            if message not in self.errors:
                self.errors.append(message)

    def open(self):
        """Attempt each selected device independently, retaining partial failure."""
        if self._opened or self._closed:
            raise RuntimeError("HardwareBackend is single-use; create a new instance")
        self._lease.acquire()  # Conflict is fatal before any device is touched.
        self._opened = True
        try:
            self.kinematics = TianjiKinematics()
        except Exception as error:
            self._open_errors["kinematics"] = f"offline kinematics: {error}"
        try:
            self.driver = TianjiDriver(self.tianji_settings["controller_ip"])
            self.driver.start()
            if self.kinematics is not None:
                self.executor = TianjiCartesianExecutor(self.driver, self.kinematics)
        except Exception as error:
            self._open_errors["tianji"] = f"Tianji connection: {error}"
        for side in self.sides:
            try:
                hand = WujiHandDriver(side, self.wuji_settings["devices"][side]["hand"],
                                      timeout_s=self.wuji_settings.get("hand_timeout_s", .5))
                self.hands[side] = hand
                hand.start()
            except Exception as error:
                self._open_errors[f"hand_{side}"] = f"{side} hand connection: {error}"

    def snapshot(self):
        now = time.monotonic_ns()
        with self._state_lock:
            problems = list(self._open_errors.values()) + list(self.errors)
            if self._fault_reason:
                problems.append(f"fault latched: {self._fault_reason}")
        if not self._opened or self._closed:
            problems.append("hardware backend is not open")
        arms = {side: dict(pose=None, joints_rad=None, velocity_rad_s=None,
                           state=None, error=None, age_s=None) for side in SIDES}
        hands = {side: dict(position_rad=None, velocity_rad_s=None, current_a=None,
                            enabled=None, age_s=None, selected=side in self.sides) for side in SIDES}
        if self.driver is not None:
            try:
                health = self.driver.health(sides=self.sides)
                if not health.ready:
                    problems.append(f"Tianji: {health.detail}")
                sample = self.driver.get_latest()
                if sample is not None:
                    for side in SIDES:
                        arm = sample.payload.arms[side]
                        stamp = getattr(self.driver, "_advanced", {}).get(side, sample.header.received_monotonic_ns)
                        data = arms[side]
                        data.update(joints_rad=arm.joints.position_rad,
                                    velocity_rad_s=arm.joints.velocity_rad_s,
                                    state=arm.state, error=arm.error,
                                    age_s=max(0., (now - stamp) / 1e9),
                                    native_current_permille=arm.native_current_permille,
                                    measured_torque_nm=arm.joints.measured_torque_nm,
                                    estimated_external_torque_nm=arm.joints.estimated_external_torque_nm)
                        joints = arm.joints.position_rad
                        if (self.kinematics is not None and len(joints) == 7 and
                                all(q is not None and math.isfinite(q) for q in joints)):
                            try:
                                data["pose"] = asdict(self.kinematics.fk(side, tuple(joints)))
                            except Exception as error:
                                data["pose_error"] = str(error)
                                if side in self.sides:
                                    problems.append(f"{side} FK: {error}")
            except Exception as error:
                problems.append(f"Tianji snapshot: {error}")
        else:
            problems.append("Tianji is unavailable")
        for side in self.sides:
            hand = self.hands.get(side)
            if hand is None:
                problems.append(f"{side} hand is unavailable")
                continue
            try:
                data = hands[side]
                sample = hand.get_latest()
                diag = hand.get_latest_stream("diagnostics")
                # The core health() latches an incomplete startup once joints
                # precede diagnostics. Wait for both first observations before
                # invoking it; genuine receiver faults remain latched in core.
                if sample is None or diag is None:
                    problems.append(f"{side} hand: waiting for joint and diagnostic feedback")
                else:
                    health = hand.health(check_latch=True)
                    if not health.ready:
                        problems.append(f"{side} hand: {health.detail}")
                if sample is not None:
                    data["age_s"] = max(0., (now - sample.header.received_monotonic_ns) / 1e9)
                    for key, attr in (("position_rad", "position_rad"), ("velocity_rad_s", "velocity_rad_s"),
                                      ("current_a", "motor_current_a")):
                        data[key] = getattr(sample.payload, attr, None)
                if diag is not None and isinstance(diag.payload, WujiDiagnostics):
                    states = diag.payload.states
                    data.update(diagnostic_states=states, error_codes=diag.payload.error_codes,
                                limit_flags=diag.payload.limit_flags,
                                diagnostics_age_s=max(0., (now - diag.header.received_monotonic_ns) / 1e9))
                    if len(states) == 20 and all(state is not None for state in states):
                        if all(state == "Enabled" for state in states):
                            data["enabled"] = True
                        elif all(state != "Enabled" for state in states):
                            data["enabled"] = False
                data["owned_enabled"] = hand.enabled
            except Exception as error:
                problems.append(f"{side} hand snapshot: {error}")
        problems = list(dict.fromkeys(problems))
        return _json_value(dict(monotonic_ns=now, healthy=not problems, problems=problems,
                                arms=arms, hands=hands, sides=self.sides,
                                details={"configured": self._configured,
                                         "local_lock_only": True,
                                         "remote_controller_exclusion": "not established"}))

    def _require_healthy(self):
        state = self.snapshot()
        if not state["healthy"]:
            raise RuntimeError("Hardware is not healthy: " + "; ".join(state["problems"]))
        return state

    def _commandable(self):
        if self._closed or not self._opened or not self._configured or self._fault_reason:
            raise RuntimeError("Hardware is not configured and available for motion")

    def configure(self):
        if self._configured:
            raise RuntimeError("Hardware is already configured")
        state = self._require_healthy()
        if any(state["hands"][side]["enabled"] is not False for side in self.sides):
            raise RuntimeError("Selected hands must report disabled before configuration")
        if any(state["arms"][side]["state"] != 0 for side in self.sides):
            raise RuntimeError("Selected arms must report IDLE before configuration")
        if self.executor is None:
            raise RuntimeError("Cartesian executor unavailable")
        try:
            with self._arm_lock:
                self.executor.configure(self.arm_profile)
            for side in self.sides:
                with self._hand_locks[side]:
                    self.hands[side].configure(self.hand_profile)
            self._configured = True
        except Exception as error:
            self._record(f"configure failed: {error}")
            raise

    def engage_hand(self, side):
        if side not in self.sides:
            raise ValueError("hand side is not selected")
        with self._hand_locks[side]:
            self._commandable()
            self._require_healthy()
            hand = self.hands[side]
            hand.engage()
            # Core engagement seeds from measurement with at most 0.5-degree
            # correction at nominal limits. Continue its exact accepted seed.
            return tuple(hand.last_target.position_rad)

    def engage_arms(self):
        with self._arm_lock:
            self._commandable()
            self._require_healthy()
            self.executor.engage()
            return self.executor.engagement_poses

    @staticmethod
    def _command_times(now_ns):
        if isinstance(now_ns, bool) or not isinstance(now_ns, int):
            raise ValueError("now_ns must be a monotonic integer timestamp")
        actual = time.monotonic_ns()
        if now_ns > actual or now_ns + COMMAND_TTL_NS <= actual:
            raise ValueError("command timestamp is future or expired")
        return now_ns, now_ns + COMMAND_TTL_NS

    def send_arms(self, poses: dict[str, Pose], now_ns):
        with self._arm_lock:
            self._commandable()
            created, expires = self._command_times(now_ns)
            sample = self.driver.get_latest()
            refs = () if sample is None else (sample.header.ref,)
            target = RobotTarget(uuid4().hex, dict(poses), refs, created, expires, self.arm_profile.profile_id)
            result = self.executor.submit(target)
            if not result.accepted:
                raise RuntimeError(f"arm command rejected: {result.reason}")

    def send_hand(self, side, joints: tuple, now_ns):
        if side not in self.sides:
            raise ValueError("hand side is not selected")
        with self._hand_locks[side]:
            self._commandable()
            created, expires = self._command_times(now_ns)
            hand = self.hands[side]
            sample = hand.get_latest()
            refs = () if sample is None else (sample.header.ref,)
            target = DeviceCommand(hand.device_id, uuid4().hex, JointTarget(JOINT_NAMES, tuple(joints)),
                                   refs, created, expires, self.hand_profile.profile_id)
            result = hand.submit(target)
            if not result.accepted:
                raise RuntimeError(f"{side} hand command rejected: {result.reason}")

    def fault_stop(self, reason):
        """Latch first, attempt every requested stop, then raise aggregate errors."""
        with self._state_lock:
            self._fault_reason = self._fault_reason or str(reason)
        errors = []
        if self.driver is not None:
            try:
                with self._arm_lock:
                    (self.executor or self.driver).request_hold(str(reason))
            except Exception as error:
                errors.append(f"arm hold failed: {error}")
        for side, hand in self.hands.items():
            try:
                with self._hand_locks[side]:
                    hand.disable(str(reason))
                    if getattr(hand, "_enable_attempted", False):
                        raise RuntimeError("disable feedback was not confirmed")
            except Exception as error:
                errors.append(f"{side} hand disable failed: {error}")
        for error in errors:
            self._record(error)
        if errors:
            raise RuntimeError("; ".join(errors))

    def close(self):
        """Caller must stop its loops first. Close includes servo-off if owned."""
        with self._state_lock:
            if self._closed:
                return
            self._closed = True
        errors = []
        if self.driver is not None:
            try:
                with self._arm_lock:
                    self.driver.close()
            except Exception as error:
                errors.append(f"Tianji close failed: {error}")
        for side, hand in self.hands.items():
            try:
                with self._hand_locks[side]:
                    hand.close()
            except Exception as error:
                errors.append(f"{side} hand close failed: {error}")
        try:
            self._lease.release()
        except Exception as error:
            errors.append(str(error))
        for error in errors:
            self._record(error)
        if errors:
            raise RuntimeError("; ".join(errors))
