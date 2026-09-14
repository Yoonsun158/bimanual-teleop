"""M6 model provenance, explicit motion configuration and local SDK kinematics.

Reading a controller export proves agreement with that file, not live agreement.
The driver separately downloads and verifies the currently connected controller.
"""

from __future__ import annotations

import configparser
import ctypes
from dataclasses import dataclass
import hashlib
import math
from pathlib import Path
import threading
from typing import Mapping

from bimanual_teleop.types import ControlProfile, Pose, Side
from bimanual_teleop.paths import PROJECT_ROOT


SDK_COMMIT = "02440e886fb59095711eb9ec6dcbedd8be08922a"
MODEL_SHA256 = "ce20b1a80974c3ed58cdac4973152625096c8ad93a779e5fd4108c26ce19bd52"
DEFAULT_MODEL = PROJECT_ROOT / "tianji_bridge/models/ccs_m6_40.MvKDCfg"
DEFAULT_LIBRARY = PROJECT_ROOT / "tianji_bridge/build/libtianji_bridge.so"
SIDES: tuple[Side, ...] = ("left", "right")


class ProfileError(ValueError):
    """A motion parameter or its verification is missing or inconsistent."""


class ModelMismatchError(ProfileError):
    """The controller export differs from the kinematics model."""


class KinematicsError(RuntimeError):
    """A local SDK kinematics operation failed or has no usable solution."""


def _number(value: object, name: str, *, minimum: float | None = None) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
        raise ProfileError(f"{name} must be a finite number")
    if minimum is not None and value < minimum:
        raise ProfileError(f"{name} must be >= {minimum}")
    return float(value)


def _vector(value: object, length: int, name: str, *, minimum: float | None = None) -> tuple[float, ...]:
    if not isinstance(value, (list, tuple)) or len(value) != length:
        raise ProfileError(f"{name} must contain {length} numbers")
    return tuple(_number(item, name, minimum=minimum) for item in value)


def _mapping(value: object, name: str) -> Mapping[str, object]:
    if not isinstance(value, Mapping):
        raise ProfileError(f"{name} must be an object")
    return value


def _text(value: object, name: str) -> str:
    if not isinstance(value, str) or not value.strip():
        raise ProfileError(f"{name} must be specified")
    return value


@dataclass(frozen=True)
class ArmModel:
    controller_type: int
    gravity_m_s2: tuple[float, ...]
    dh_native: tuple[tuple[float, ...], ...]  # alpha deg, a mm, d mm, theta deg; flange last
    limits_native: tuple[tuple[float, ...], ...]  # positive/negative deg, velocity deg/s, acceleration deg/s²
    bd67_native: tuple[tuple[float, ...], ...]  # PP, NP, NN, PN, following the SDK file
    dof: int = 7

    @property
    def lower_rad(self) -> tuple[float, ...]:
        return tuple(math.radians(row[1]) for row in self.limits_native)

    @property
    def upper_rad(self) -> tuple[float, ...]:
        return tuple(math.radians(row[0]) for row in self.limits_native)

    @property
    def max_joint_velocity_rad_s(self) -> tuple[float, ...]:
        return tuple(math.radians(row[2]) for row in self.limits_native)


@dataclass(frozen=True)
class ModelVerification:
    source: str
    export_path: str
    export_digest: str
    nominal_digest: str
    sides: tuple[Side, ...]
    connected_export_checked: bool = False
    live_parameters_verified: bool = False

    @property
    def live_verified(self) -> bool:
        """An exported robot.ini cannot establish unsaved live parameter values."""
        return self.live_parameters_verified


@dataclass(frozen=True)
class M6Model:
    path: Path
    digest: str
    arms: Mapping[Side, ArmModel]
    sdk_commit: str = SDK_COMMIT

    @classmethod
    def from_file(cls, path: str | Path = DEFAULT_MODEL) -> M6Model:
        path = Path(path).resolve()
        raw = path.read_bytes()
        digest = hashlib.sha256(raw).hexdigest()
        if digest != MODEL_SHA256:
            raise ProfileError("This version requires the pinned M6 4.0 model; a path override only relocates that file")
        try:
            rows = [tuple(float(item) for item in line.split(",") if item.strip())
                    for line in raw.decode("utf-8-sig").splitlines() if line.strip()]
        except (ValueError, UnicodeError) as error:
            raise ProfileError(f"Invalid M6 model file: {path}") from error
        if len(rows) != 26:
            raise ProfileError("M6 model must contain two 13-row arm records")
        arms = {}
        for side, start in zip(SIDES, (0, 13)):
            block = rows[start:start + 13]
            if ([len(row) for row in block] != [4] + [18] * 7 + [4] + [3] * 4
                    or block[0][0] != 1017 or not all(math.isfinite(x) for row in block for x in row)):
                raise ProfileError(f"Invalid M6 4.0 geometry for {side}")
            limits = tuple((row[5], row[4], row[6], row[7]) for row in block[1:8])
            if any(positive <= negative or velocity <= 0 or acceleration <= 0
                   for positive, negative, velocity, acceleration in limits):
                raise ProfileError(f"Invalid joint limits for {side}")
            arms[side] = ArmModel(1017, block[0][1:], tuple(row[:4] for row in block[1:9]),
                                 limits, tuple(block[9:13]))
        return cls(path, digest, arms)

    def arm(self, side: Side) -> ArmModel:
        if side not in SIDES:
            raise ProfileError(f"Unknown arm: {side}")
        return self.arms[side]

    def verify_controller_export(
        self, path: str | Path, sides: tuple[Side, ...] = SIDES, *, source: str = "provided_export",
    ) -> ModelVerification:
        if source not in ("provided_export", "downloaded_controller_export"):
            raise ProfileError("Unknown controller export provenance")
        path = Path(path).resolve()
        raw = path.read_bytes()
        parser = configparser.ConfigParser(interpolation=None)
        try:
            parser.read_string(raw.decode("utf-8-sig"))
        except (UnicodeError, configparser.Error) as error:
            raise ProfileError(f"Invalid controller export: {path}") from error
        mismatches = []
        for side in sides:
            arm = self.arm(side)
            prefix = f"R.A{SIDES.index(side)}"
            expected = [(f"{prefix}.BASIC", "Type", arm.controller_type),
                        (f"{prefix}.BASIC", "Dof", arm.dof)]
            for index, dh in enumerate(arm.dh_native):
                section = f"{prefix}.L{index}.DH" if index < 7 else f"{prefix}.FLANGE"
                expected.extend((section, key, value) for key, value in zip(("Alpha", "A", "D", "Theta"), dh))
            for index, limits in enumerate(arm.limits_native):
                expected.extend((f"{prefix}.L{index}.BASIC", key, value) for key, value in
                                zip(("LimitPos", "LimitNeg", "VelMax", "AccMax"), limits))
            for quadrant, coefficients in zip(("PP", "NP", "NN", "PN"), arm.bd67_native):
                expected.extend((f"{prefix}.CTRL", f"BD67{quadrant}{index}", value)
                                for index, value in enumerate(coefficients))
            for section, key, value in expected:
                try:
                    actual = parser.getfloat(section, key)
                except (ValueError, configparser.Error) as error:
                    raise ProfileError(f"Missing/invalid controller parameter {section}.{key}") from error
                if not math.isclose(actual, value, rel_tol=1e-9, abs_tol=1e-6):
                    mismatches.append(f"{section}.{key}: export {actual}, model {value}")
        if mismatches:
            raise ModelMismatchError("Controller/model mismatch: " + "; ".join(mismatches))
        return ModelVerification(source, str(path), hashlib.sha256(raw).hexdigest(), self.digest,
                                 tuple(sides), connected_export_checked=source == "downloaded_controller_export")


@dataclass(frozen=True)
class ArmMotionProfile:
    stiffness: tuple[float, ...]
    damping: tuple[float, ...]
    nullspace_stiffness: float
    nullspace_damping: float
    tool_dyn10: tuple[float, ...]
    velocity_ratio: int
    acceleration_ratio: int


@dataclass(frozen=True)
class MotionProfile:
    profile_id: str
    active_arms: tuple[Side, ...]
    arms: Mapping[Side, ArmMotionProfile]
    model: M6Model

    @classmethod
    def from_control_profile(cls, profile: ControlProfile, model_path: str | Path | None = None) -> MotionProfile:
        if profile.mode != "cartesian_impedance":
            raise ProfileError("M6 motion requires mode='cartesian_impedance'")
        _text(profile.profile_id, "profile_id")
        parameters = _mapping(profile.parameters, "parameters")
        active = parameters.get("active_arms")
        if (not isinstance(active, (list, tuple)) or not active or
                any(side not in SIDES for side in active) or len(set(active)) != len(active)):
            raise ProfileError("active_arms must explicitly name left, right, or both without duplicates")
        model = M6Model.from_file(model_path or DEFAULT_MODEL)
        arm_parameters = _mapping(parameters.get("arms"), "arms")
        arms = {}
        for side in active:
            config = _mapping(arm_parameters.get(side), f"arms.{side}")
            ratios = []
            for key in ("velocity_ratio", "acceleration_ratio"):
                ratio = config.get(key)
                if isinstance(ratio, bool) or not isinstance(ratio, int) or not 1 <= ratio <= 100:
                    raise ProfileError(f"{side}.{key} must be an integer from 1 to 100")
                ratios.append(ratio)
            dynamics = _vector(config.get("tool_dyn10"), 10, f"{side}.tool_dyn10")
            if dynamics[0] < 0:
                raise ProfileError(f"{side} tool load mass must be nonnegative")
            if dynamics[0] == 0 and any(value != 0 for value in dynamics[1:]):
                raise ProfileError(f"{side} unloaded tool dynamics must contain ten zeros")
            damping = _vector(config.get("damping"), 6, f"{side}.damping", minimum=0)
            nullspace_damping = _number(config.get("nullspace_damping"), f"{side}.nullspace_damping", minimum=0)
            if any(value > 1 for value in (*damping, nullspace_damping)):
                raise ProfileError(f"{side} damping and nullspace_damping must be within [0, 1]")
            arms[side] = ArmMotionProfile(
                _vector(config.get("stiffness"), 6, f"{side}.stiffness", minimum=0),
                damping,
                _number(config.get("nullspace_stiffness"), f"{side}.nullspace_stiffness", minimum=0),
                nullspace_damping,
                dynamics, *ratios,
            )
        return cls(profile.profile_id, tuple(active), arms, model)


class _IkResult(ctypes.Structure):
    _fields_ = [("q", ctypes.c_double * 7), ("solution_count", ctypes.c_int32),
                ("out_of_range", ctypes.c_int32), ("singular_mask", ctypes.c_int32),
                ("limit_mask", ctypes.c_int32)]


_KINE_LOCK = threading.RLock()
_LOADED_MODELS: dict[tuple[str, Side], str] = {}
_DOUBLE_PTR = ctypes.POINTER(ctypes.c_double)


def _array(values):
    return (ctypes.c_double * len(values))(*values)


def _matrix_from_pose(pose: Pose, side: Side) -> tuple[float, ...]:
    if pose.parent_frame != f"tianji_{side}_base" or pose.child_frame != f"tianji_{side}_flange":
        raise KinematicsError("Pose must express this arm's flange in its base frame")
    try:
        position = _vector(pose.position_m, 3, "position_m")
        x, y, z, w = _vector(pose.orientation_xyzw, 4, "orientation_xyzw")
    except ProfileError as error:
        raise KinematicsError(str(error)) from error
    if not math.isclose(x*x + y*y + z*z + w*w, 1, abs_tol=1e-6):
        raise KinematicsError("Pose quaternion must have unit norm")
    return (1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w), position[0]*1000,
            2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w), position[1]*1000,
            2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y), position[2]*1000,
            0, 0, 0, 1)


def _pose_from_matrix(matrix, side: Side) -> Pose:
    m = matrix
    if not all(math.isfinite(value) for value in m):
        raise KinematicsError("FK returned a nonfinite transform")
    trace = m[0] + m[5] + m[10]
    if trace > 0:
        scale = math.sqrt(trace + 1) * 2
        q = ((m[9]-m[6])/scale, (m[2]-m[8])/scale, (m[4]-m[1])/scale, scale/4)
    else:
        i = max(range(3), key=lambda index: m[index*5])
        j, k = (i+1) % 3, (i+2) % 3
        scale = math.sqrt(1 + m[i*5] - m[j*5] - m[k*5]) * 2
        q = [0.0]*4
        q[i], q[j], q[k] = scale/4, (m[j*4+i]+m[i*4+j])/scale, (m[k*4+i]+m[i*4+k])/scale
        q[3] = (m[k*4+j]-m[j*4+k])/scale
    norm = math.sqrt(sum(value*value for value in q))
    return Pose(f"tianji_{side}_base", f"tianji_{side}_flange",
                (m[3]/1000, m[7]/1000, m[11]/1000), tuple(value/norm for value in q))


class TianjiKinematics:
    """Local official FK/IK; constructing this object explicitly loads the bridge."""

    def __init__(self, library_path: str | Path | None = None, model_path: str | Path | None = None):
        self.model = M6Model.from_file(model_path or DEFAULT_MODEL)
        self.library_path = Path(library_path or DEFAULT_LIBRARY).resolve()
        self._lib = ctypes.CDLL(str(self.library_path))
        self._lib.tj_abi_version.restype = ctypes.c_int
        self._lib.tj_last_error.restype = ctypes.c_char_p
        if self._lib.tj_abi_version() != 2:
            raise KinematicsError("Unsupported Tianji bridge ABI")
        self._lib.tj_init_kine.argtypes = [ctypes.c_int32, ctypes.c_int32] + [_DOUBLE_PTR]*4
        self._lib.tj_init_kine.restype = ctypes.c_int
        self._lib.tj_fk.argtypes = [ctypes.c_int32, _DOUBLE_PTR, _DOUBLE_PTR]
        self._lib.tj_fk.restype = ctypes.c_int
        self._lib.tj_ik.argtypes = [ctypes.c_int32, _DOUBLE_PTR, _DOUBLE_PTR, ctypes.POINTER(_IkResult)]
        self._lib.tj_ik.restype = ctypes.c_int

    def _check(self, operation: str, result: int) -> None:
        if result != 0:
            detail = self._lib.tj_last_error()
            raise KinematicsError(f"{operation}: {detail.decode(errors='replace') if detail else result}")

    def _initialize(self, side: Side) -> int:
        if side not in SIDES:
            raise KinematicsError(f"Unknown arm: {side}")
        arm = self.model.arm(side)
        key = (str(self.library_path), side)
        index = SIDES.index(side)
        if _LOADED_MODELS.get(key) != self.model.digest:
            identity = _array((1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1))
            self._check("initialize kinematics", self._lib.tj_init_kine(
                index, arm.controller_type, _array(sum(arm.dh_native, ())),
                _array(sum(arm.limits_native, ())), _array(sum(arm.bd67_native, ())), identity))
            _LOADED_MODELS[key] = self.model.digest
        return index

    @staticmethod
    def _joints(joints_rad) -> tuple[float, ...]:
        try:
            return _vector(joints_rad, 7, "joints_rad")
        except ProfileError as error:
            raise KinematicsError(str(error)) from error

    def fk(self, side: Side, joints_rad: tuple[float, ...]) -> Pose:
        joints = _array(tuple(math.degrees(value) for value in self._joints(joints_rad)))
        matrix = _array((0.0,)*16)
        with _KINE_LOCK:
            self._check("FK", self._lib.tj_fk(self._initialize(side), joints, matrix))
        return _pose_from_matrix(matrix, side)

    def ik(self, side: Side, pose: Pose, reference_rad: tuple[float, ...]) -> tuple[float, ...]:
        matrix = _array(_matrix_from_pose(pose, side))
        reference = _array(tuple(math.degrees(value) for value in self._joints(reference_rad)))
        result = _IkResult()
        with _KINE_LOCK:
            self._check("IK", self._lib.tj_ik(self._initialize(side), matrix, reference, ctypes.byref(result)))
        if result.solution_count <= 0:
            raise KinematicsError("IK target is unreachable")
        if result.out_of_range or result.limit_mask:
            exceeded = ", ".join(f"J{i+1}={q:.2f}deg" for i, q in enumerate(result.q)
                                  if result.limit_mask & (1 << i))
            raise KinematicsError(f"{side}: IK violates joint/coupled limits ({exceeded or 'unreachable target'})")
        if result.singular_mask:
            raise KinematicsError(f"IK is singular (mask={result.singular_mask})")
        joints = tuple(math.radians(value) for value in result.q)
        arm = self.model.arm(side)
        if any(not math.isfinite(q) or q < low-1e-9 or q > high+1e-9
               for q, low, high in zip(joints, arm.lower_rad, arm.upper_rad)):
            raise KinematicsError("IK returned joints outside model limits")
        return joints
