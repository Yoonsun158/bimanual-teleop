"""Move each Tianji arm once to the teleoperation initial joint pose.

Confirm the surroundings are safe and press Enter to move the selected arms.
--inspect reads the current pose without moving.
--reset explicitly confirms physical emergency release before resetting fault 13.
"""
from __future__ import annotations

import argparse
import math
from pathlib import Path
import time

from bimanual_teleop.common.console import configure_runtime_logging, print_message
from bimanual_teleop.common.terminal import NonblockingTerminal, confirm_motion
from bimanual_teleop.devices.tianji.driver import TianjiDriver
from bimanual_teleop.devices.tianji.config import DEFAULT_CONFIG, load_config
from bimanual_teleop.types import ControlProfile


SIDES = ("left", "right")


def load_targets(source, side="both"):
    if side not in (*SIDES, "both"):
        raise ValueError("side must be left, right, or both")
    order = source.get("order", list(SIDES))
    if side == "both" and (not isinstance(order, list) or len(order) != 2 or set(order) != set(SIDES)):
        raise ValueError("order must contain left and right exactly once")
    selected = tuple(order) if side == "both" else (side,)
    targets = {}
    for arm in selected:
        values = source["target_deg"][arm]
        if (not isinstance(values, list) or len(values) != 7 or any(
                isinstance(q, bool) or not isinstance(q, (int, float)) or not math.isfinite(q)
                for q in values)):
            raise ValueError(f"{arm} target requires seven finite joint angles in degrees")
        targets[arm] = tuple(map(math.radians, values))
    return source, selected, targets


def await_feedback(driver):
    deadline = time.monotonic() + 2
    while (sample := driver.get_latest()) is None:
        if time.monotonic() >= deadline:
            raise RuntimeError("No Tianji feedback received")
        time.sleep(.01)
    return sample


def run(args, *, confirmed=False):
    """Position the arms; an internal worker may inherit its parent's confirmation."""
    side = getattr(args, "side", "both")
    driver = None
    error = None
    try:
        configure_runtime_logging()
        settings = load_config(args.config, args.ip)
        controller_ip = settings["controller_ip"]
        source, order, targets = load_targets(settings["ready_pose"], side=side)
        profile_source = settings["profile"]
        profile_source["parameters"]["active_arms"] = list(order)
        if side != "both":
            profile_source["profile_id"] += f"-{side}"
        for arm in order:
            for key in ("velocity_ratio", "acceleration_ratio"):
                if key in source:
                    profile_source["parameters"]["arms"][arm][key] = source[key]
        profile_source["profile_id"] += "-ready"
        profile = ControlProfile(**profile_source)
        if not args.inspect and not confirmed:
            with NonblockingTerminal() as terminal:
                if not confirm_motion(terminal, "开始机械臂回位"):
                    return 0
        driver = TianjiDriver(controller_ip, args.library, model_path=args.model)
        driver.start()
        initial = await_feedback(driver)
        if args.inspect:
            for selected_side in order:
                label = "左臂" if selected_side == "left" else "右臂"
                arm = initial.payload.arms[selected_side]
                angles = [round(math.degrees(q), 2) for q in arm.joints.position_rad]
                print_message(f"{label}：{angles}° · 状态 {arm.state} · 错误 {arm.error}",
                              "error" if arm.error else "info")
        if args.reset and any(initial.payload.arms[arm].error == 13 for arm in order):
            driver.reset_released_emergency(physical_release_confirmed=True)
        if not args.inspect:
            driver.configure(profile)
            for arm in order:
                label = "左臂" if arm == "left" else "右臂"
                ratio = profile.parameters["arms"][arm]["velocity_ratio"]
                print_message(f"{label}回位中 · 速度 {ratio}%")
                driver.move_joints(arm, targets[arm])
            print_message(("双臂" if side == "both" else "左臂" if side == "left" else "右臂") +
                          "初始姿态准备完成", "done")
    except BaseException as exc:
        error = f"{type(exc).__name__}: {exc}"
    finally:
        if driver is not None:
            try:
                driver.close()
            except BaseException as exc:
                error = error or f"{type(exc).__name__}: {exc}"
        if error:
            print_message(f"初始定位失败：{error}", "error")
    return int(error is not None)


def parser():
    result = argparse.ArgumentParser(description=__doc__)
    result.add_argument("--tianji-config", "--config", dest="config", type=Path, default=DEFAULT_CONFIG,
                        help="天机统一配置 YAML；默认 configs/tianji_teleop.yaml")
    result.add_argument("--ip", help="临时覆盖设备配置中的天机控制器 IP")
    result.add_argument("--library", type=Path, help="built libtianji_bridge.so")
    result.add_argument("--model", type=Path, help="Tianji kinematics model")
    result.add_argument("--side", choices=("left", "right", "both"), default="both")
    result.add_argument("--reset", action="store_true", help="confirm physical emergency release and reset fault 13")
    result.add_argument("--inspect", action="store_true", help="read the current pose without moving")
    return result


if __name__ == "__main__":
    raise SystemExit(run(parser().parse_args()))
