"""Replay Tianji IK diagnostics locally; this module has no device transport.

Run with the project's Conda environment. Only the official libKine.so is loaded;
the control SDK, driver, and Quest acquisition are not imported.
"""

from __future__ import annotations

import argparse
from dataclasses import asdict
import hashlib
import json
import math
from pathlib import Path
import sys

from bimanual_teleop.devices.tianji.model import (
    DEFAULT_MODEL, MODEL_SHA256, SDK_COMMIT, TianjiKinematics,
    _matrix_from_pose,
)
from bimanual_teleop.types import Pose
from bimanual_teleop.devices.tianji.sdk import add_sdk_argument


SCHEMA = "tianji_ik_failure_v1"
MARKER = "[IK诊断]"
LIMITATION = "仅为离线单帧候选；不保证轨迹连续、关节速度可行或碰撞安全，不得直接作为设备命令。"
UPSTREAM = "https://github.com/wuji-technology/wuji-hand-teleop/blob/647801345a6a27dec5cbf56280ce63bb8b2f6a32/"
def _vector(value, size, name, *, nullable=False):
    if nullable and value is None:
        return None
    if not isinstance(value, (list, tuple)) or len(value) != size:
        raise ValueError(f"{name} must contain {size} numbers")
    result = []
    for item in value:
        if nullable and item is None:
            result.append(None)
        elif type(item) not in (float, int):
            raise ValueError(f"{name} must contain finite numbers")
        else:
            try:
                number = float(item)
            except OverflowError as error:
                raise ValueError(f"{name} must contain finite numbers") from error
            if not math.isfinite(number):
                raise ValueError(f"{name} must contain finite numbers")
            result.append(number)
    return tuple(result)


def validate_diagnostic(value, *, model_sha256=MODEL_SHA256):
    if not isinstance(value, dict):
        raise ValueError("diagnostic must be a JSON object")
    required = {"schema", "sdk_commit", "model_sha256", "side", "target", "reference_deg",
                "result_deg", "status", "solution_count", "out_of_range", "singular_mask", "limit_mask"}
    missing = required - value.keys()
    if missing:
        raise ValueError("missing diagnostic fields: " + ", ".join(sorted(missing)))
    if value["schema"] != SCHEMA:
        raise ValueError(f"expected schema {SCHEMA}")
    if value["sdk_commit"] != SDK_COMMIT:
        raise ValueError(f"SDK commit mismatch: expected {SDK_COMMIT}")
    if value["model_sha256"] != model_sha256:
        raise ValueError(f"model SHA256 mismatch: expected {model_sha256}")
    side = value["side"]
    if side not in ("left", "right"):
        raise ValueError("side must be left or right")
    target = value["target"]
    if not isinstance(target, dict) or not {
            "parent_frame", "child_frame", "position_m", "orientation_xyzw"} <= target.keys():
        raise ValueError("target requires parent_frame, child_frame, position_m, orientation_xyzw")
    pose = Pose(target["parent_frame"], target["child_frame"],
                _vector(target["position_m"], 3, "target.position_m"),
                _vector(target["orientation_xyzw"], 4, "target.orientation_xyzw"))
    _matrix_from_pose(pose, side)  # Validate frame names and quaternion norm.
    reference = _vector(value["reference_deg"], 7, "reference_deg")
    result = _vector(value["result_deg"], 7, "result_deg", nullable=True)
    if value["status"] is None:
        raise ValueError("IK was not called (status=null); inspect model/SDK initialization, not a solver result")
    if type(value["status"]) is not int:
        raise ValueError("status must be an integer")
    for key in ("solution_count", "singular_mask", "limit_mask"):
        if type(value[key]) is not int or value[key] < 0:
            raise ValueError(f"{key} must be a nonnegative integer")
    if value["singular_mask"] > 127 or value["limit_mask"] > 127:
        raise ValueError("joint masks must fit seven joints")
    if type(value["out_of_range"]) not in (bool, int) or value["out_of_range"] not in (0, 1):
        raise ValueError("out_of_range must be boolean or 0/1")
    return {**value, "target": asdict(pose), "reference_deg": reference, "result_deg": result}


def read_records(text):
    """Read a JSON object/array/JSONL or marked records amid console output.

    raw_decode deliberately permits the UI's natural-language suffix after a
    marked JSON object. A broken record stays visible without hiding later ones.
    """
    decoder = json.JSONDecoder()
    try:
        value = json.loads(text)
    except (ValueError, TypeError):
        records = []
        for number, line in enumerate(text.splitlines(), 1):
            marked = MARKER in line
            payload = line.split(MARKER, 1)[1].strip() if marked else line.strip()
            if not marked and not payload.startswith("{"):
                continue
            try:
                value, end = decoder.raw_decode(payload)
                if not marked and payload[end:].strip():
                    raise ValueError("unexpected text after JSON object")
                records.append({"line": number, "diagnostic": value})
            except ValueError as error:
                records.append({"line": number, "error": f"invalid diagnostic JSON: {error}"})
        if not records:
            raise ValueError("no JSON diagnostics or [IK诊断] records found")
        return records
    return [{"diagnostic": item} for item in (value if isinstance(value, list) else [value])]


class OfflineKinematics(TianjiKinematics):
    """Reuse official kinematics without ever loading the control SDK."""

    def __init__(self, sdk_root=None, model_path=DEFAULT_MODEL):
        super().__init__(sdk_root, model_path)
        self.library_sha256 = hashlib.sha256(self.library_path.read_bytes()).hexdigest()

    def fk(self, side, joints_deg):
        return super().fk(side, tuple(math.radians(q) for q in _vector(joints_deg, 7, "joints_deg")))

    def _solve(self, diagnostic, *, direction=None, angle=None):
        success, solve = self.solve(diagnostic["side"], Pose(**diagnostic["target"]),
                                    tuple(math.radians(q) for q in diagnostic["reference_deg"]),
                                    direction=direction, angle=angle)
        return self._summarize(diagnostic, solve, success)

    def _summarize(self, diagnostic, solve, success):
        q = tuple(solve.m_Output_RetJoint.data)
        finite = all(math.isfinite(value) for value in q)
        side, pose = diagnostic["side"], Pose(**diagnostic["target"])
        arm = self.model.arm(side)
        model_violations = [i + 1 for i, (value, limits) in enumerate(zip(q, arm.limits_native))
                            if not math.isfinite(value) or value < limits[1]-1e-9 or value > limits[0]+1e-9]
        effective = all(math.isfinite(low) and math.isfinite(high) and high > low
                        for low, high in zip(solve.m_Output_RunLmtN.data, solve.m_Output_RunLmtP.data))
        effective_violations = [i + 1 for i, (value, low, high) in enumerate(zip(q, solve.m_Output_RunLmtN.data, solve.m_Output_RunLmtP.data))
                               if not math.isfinite(value) or value < low-1e-9 or value > high+1e-9] if effective else None
        position_error = rotation_error = None
        if finite and solve.m_OutPut_Result_Num > 0:
            recovered = self.fk(side, q)
            position_error = math.dist(recovered.position_m, pose.position_m)
            dot = abs(sum(a*b for a, b in zip(recovered.orientation_xyzw, pose.orientation_xyzw)))
            rotation_error = math.degrees(2 * math.acos(min(1., dot)))
        singular = sum(bool(flag) << i for i, flag in enumerate(solve.m_Output_IsDeg))
        limits = sum(bool(flag) << i for i, flag in enumerate(solve.m_Output_JntExdTags))
        legal = bool(success and solve.m_OutPut_Result_Num > 0 and finite and not solve.m_Output_IsOutRange
                     and not singular and not limits and not solve.m_Output_IsJntExd
                     and not model_violations and effective and not effective_violations
                     and position_error is not None and position_error <= 1e-6
                     and rotation_error <= math.degrees(1e-5))
        return {
            "status": 0 if success else -1, "solution_count": solve.m_OutPut_Result_Num,
            "out_of_range": bool(solve.m_Output_IsOutRange), "singular_mask": singular, "limit_mask": limits,
            "sdk_joint_limit_exceeded": bool(solve.m_Output_IsJntExd), "result_deg": q if finite else None,
            "legal_single_pose": legal,
            "max_joint_delta_deg": max(abs(a-b) for a, b in zip(q, diagnostic["reference_deg"])) if finite else None,
            "model_limit_violations": model_violations, "effective_limit_violations": effective_violations,
            "minimum_effective_limit_margin_deg": min(min(value-low, high-value)
                for value, low, high in zip(q, solve.m_Output_RunLmtN.data, solve.m_Output_RunLmtP.data)) if finite and effective else None,
            "fk_position_error_m": position_error, "fk_orientation_error_deg": rotation_error,
        }

    def analyze(self, value, *, scan_nsp=False, scan_step_deg=5):
        diagnostic = validate_diagnostic(value, model_sha256=self.model.digest)
        if scan_step_deg not in (1, 2, 5, 10, 15, 30, 45, 90):
            raise ValueError("scan_step_deg must be one of 1, 2, 5, 10, 15, 30, 45, 90")
        sign = -1 if diagnostic["side"] == "left" else 1
        candidates = [{"method": "NEAR_REF", "singularity_tolerance_deg": [0, 0, 0],
                       **self._solve(diagnostic)}]
        for name, direction, source in (
                ("NEAR_DIR_PICO", (0, sign * 0.5, -0.3),
                 "src/output_devices/tianji_world_output/config/tianji_robot.yaml#L105-L127"),
                ("NEAR_DIR_HTC", (0, sign, -0.5),
                 "src/output_devices/tianji_output/tianji_output/tianji_chest_driver.py#L153-L157")):
            candidates.append({"method": name, "direction": direction, "source": UPSTREAM + source,
                               "singularity_tolerance_deg": [5, 5, 5],
                               **self._solve(diagnostic, direction=direction)})
        replay = candidates[0]
        recorded = diagnostic["result_deg"]
        delta = max(abs(a-b) for a, b in zip(recorded, replay["result_deg"])) if (
            recorded is not None and all(value is not None for value in recorded)
            and replay["result_deg"] is not None) else None
        report = {
            "schema": "tianji_ik_analysis_v1", "side": diagnostic["side"], "limitation": LIMITATION,
            "sdk_commit": SDK_COMMIT, "model_sha256": self.model.digest,
            "kinematics_library": str(self.library_path), "library_sha256": self.library_sha256,
            "binary_provenance": "unmodified official distribution; SHA-256 verified against bundled manifest",
            "captured_diagnostic": diagnostic,
            "replay_matches_captured_flags": all(int(replay[key]) == int(diagnostic[key]) for key in
                ("status", "solution_count", "out_of_range", "singular_mask", "limit_mask")),
            "replay_result_max_difference_deg": delta, "candidates": candidates,
        }
        if scan_nsp:
            scanned = [{"method": "NSP", "angle_deg": angle, **self._solve(diagnostic, angle=angle)}
                       for angle in range(-180, 181, scan_step_deg)]
            legal = [row for row in scanned if row["legal_single_pose"]]
            report["nsp_scan"] = {"range_deg": [-180, 180], "step_deg": scan_step_deg,
                "examined": len(scanned), "legal_count": len(legal),
                "nearest_legal_candidates": sorted(legal, key=lambda row: row["max_joint_delta_deg"])[:3],
                "largest_margin_candidate": max(legal, key=lambda row: row["minimum_effective_limit_margin_deg"])
                    if legal else None}
        return report


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("input", help="JSON or console log file; - reads stdin")
    add_sdk_argument(parser)
    parser.add_argument("--model", type=Path, default=DEFAULT_MODEL)
    parser.add_argument("--scan-nsp", action="store_true", help="explore offline candidates only")
    parser.add_argument("--scan-step-deg", type=int, choices=(1, 2, 5, 10, 15, 30, 45, 90), default=5)
    args = parser.parse_args(argv)
    try:
        text = sys.stdin.read() if args.input == "-" else Path(args.input).read_text(encoding="utf-8-sig")
        records = read_records(text)
        if not records:
            raise ValueError("no diagnostic records found")
        analyzer, failed = None, False
        for number, record in enumerate(records, 1):
            try:
                if "error" in record:
                    raise ValueError(record["error"])
                diagnostic = validate_diagnostic(record["diagnostic"])
                if analyzer is None:
                    analyzer = OfflineKinematics(args.sdk_root, args.model)
                output = analyzer.analyze(diagnostic, scan_nsp=args.scan_nsp, scan_step_deg=args.scan_step_deg)
            except (ValueError, RuntimeError, OSError, AttributeError) as error:
                output, failed = {"error": str(error)}, True
            print(json.dumps({"record": number, **({"line": record["line"]} if "line" in record else {}),
                              **output}, ensure_ascii=False, allow_nan=False))
        return 1 if failed else 0
    except (ValueError, OSError) as error:
        print(f"IK offline analysis: {error}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
