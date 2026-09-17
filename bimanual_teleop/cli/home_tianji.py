"""Move each Tianji arm once to the teleoperation initial joint pose.

Confirm the surroundings are safe and press Enter to move the selected arms.
--inspect reads the current pose without moving.
"""
from __future__ import annotations

import argparse
import math
from pathlib import Path

from bimanual_teleop.devices.tianji.sdk import add_sdk_argument
from bimanual_teleop.common.console import configure_runtime_logging, print_message, runtime_message
from bimanual_teleop.common.terminal import NonblockingTerminal, confirm_motion
from bimanual_teleop.devices.tianji.driver import TianjiDriver
from bimanual_teleop.devices.tianji.config import DEFAULT_CONFIG, load_config
from bimanual_teleop.types import ControlProfile
from bimanual_teleop.control.arm.preparation import (
    load_targets, await_feedback, ready_profile, move_to_ready_pose,
)


def run(args, *, confirmed=False):
    """Position the arms; an internal worker may inherit its parent's confirmation."""
    side = getattr(args, "side", "both")
    verbose = getattr(args, "verbose", False)
    driver = None
    error = None
    try:
        configure_runtime_logging(verbose=verbose)
        settings = load_config(args.config, args.ip)
        controller_ip = settings["controller_ip"]
        source, order, targets = load_targets(settings["ready_pose"], side=side)
        profile = ready_profile(ControlProfile(**settings["profile"]), source, order)
        if not args.inspect and not confirmed:
            with NonblockingTerminal() as terminal:
                if not confirm_motion(terminal, "开始机械臂回位"):
                    return 0
        driver = TianjiDriver(controller_ip, args.sdk_root, model_path=args.model)
        driver.start()
        initial = await_feedback(driver)
        if args.inspect:
            for selected_side in order:
                label = "左臂" if selected_side == "left" else "右臂"
                arm = initial.payload.arms[selected_side]
                angles = [round(math.degrees(q), 2) for q in arm.joints.position_rad]
                print_message(f"{label}：{angles}° · 状态 {arm.state} · 错误 {arm.error}",
                              "error" if arm.error else "info")
        if not args.inspect:
            move_to_ready_pose(driver, profile, targets)
    except BaseException as exc:
        error = f"{type(exc).__name__}: {exc}"
    finally:
        if driver is not None:
            try:
                driver.close()
            except BaseException as exc:
                error = error or f"{type(exc).__name__}: {exc}"
        if error:
            print_message(runtime_message(f"初始定位失败：{error}", verbose=verbose), "error")
    if error is None and not args.inspect:
        print_message(("双臂" if side == "both" else "左臂" if side == "left" else "右臂") +
                      "初始姿态准备完成", "done")
    return int(error is not None)


def parser():
    result = argparse.ArgumentParser(description=__doc__)
    result.add_argument("--tianji-config", "--config", dest="config", type=Path, default=DEFAULT_CONFIG,
                        help="天机统一配置 YAML；默认 configs/tianji_teleop.yaml")
    result.add_argument("--ip", help="临时覆盖设备配置中的天机控制器 IP")
    add_sdk_argument(result)
    result.add_argument("--model", type=Path, help="Tianji kinematics model")
    result.add_argument("--side", choices=("left", "right", "both"), default="both")
    result.add_argument("--inspect", action="store_true", help="read the current pose without moving")
    result.add_argument("-v", "--verbose", action="store_true", help="输出完整故障诊断和调试日志")
    return result


def main(argv=None):
    return run(parser().parse_args(argv))


if __name__ == "__main__":
    raise SystemExit(main())
