"""Shared ready-pose movement and cancellable startup preparation."""

from copy import deepcopy
from dataclasses import replace
import math
from pathlib import Path
import signal
import subprocess
import sys
import time

from bimanual_teleop.common.console import print_message
from bimanual_teleop.paths import PROJECT_ROOT


ROOT = PROJECT_ROOT
SIDES = ("left", "right")


def check_preparation_input(terminal, timeout=0.):
    """Discard Enter during preparation; cancellation also covers terminal EOF."""
    keys = terminal.read(timeout)
    if keys == "" or (keys and any(key.lower() in (" ", "q", "\x03", "\x04") for key in keys)):
        raise RuntimeError("初始位姿准备已取消，控制未启动")
    return keys is not None


def stop_preparation(process):
    """Let the position runner request stop and close its SDK first."""
    if process.poll() is None:
        print_message("正在中止初始位姿准备……", "warning")
        try:
            process.send_signal(signal.SIGINT)
        except ProcessLookupError:
            pass
    while True:
        try:
            process.wait(timeout=.1)
            return
        except (subprocess.TimeoutExpired, KeyboardInterrupt):
            # Repeated Ctrl+C must not abandon the child's hold/close sequence.
            continue


def prepare_initial_pose(*, config, ip, terminal, side="both", sdk_root=None, model=None,
                         verbose=False):
    """Run after the caller has received the operator's motion confirmation."""
    config = Path(config).resolve()
    worker = ("from bimanual_teleop.cli.home_tianji import parser, run; "
              "raise SystemExit(run(parser().parse_args(), confirmed=True))")
    command = [sys.executable, "-c", worker,
               "--ip", ip, "--tianji-config", str(config), "--side", side]
    if verbose:
        command.append("--verbose")
    for flag, path in (("--sdk-root", sdk_root), ("--model", model)):
        if path is not None:
            command += [flag, str(Path(path).resolve())]
    check_preparation_input(terminal)
    label = "双臂" if side == "both" else "左臂" if side == "left" else "右臂"
    print_message(f"先将{label}移到初始位姿；Space / Q / Ctrl+C 可中止。")
    # The child releases its driver before the caller opens a control driver.
    process = subprocess.Popen(command, cwd=ROOT, stdin=subprocess.DEVNULL,
                               start_new_session=True)
    try:
        while True:
            check_preparation_input(terminal, .05)
            code = process.poll()
            if code is not None:
                break
        process.wait()
        while check_preparation_input(terminal):
            pass
    except BaseException:
        stop_preparation(process)
        raise
    if code != 0:
        raise RuntimeError(f"初始位姿准备失败（退出码 {code}），控制未启动")
    return {"side": side, "config": str(config)}


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


def ready_profile(profile, source, order):
    parameters = deepcopy(profile.parameters)
    parameters["active_arms"] = list(order)
    parameters["arms"] = {side: parameters["arms"][side] for side in order}
    for arm in order:
        for key in ("velocity_ratio", "acceleration_ratio"):
            if key in source:
                parameters["arms"][arm][key] = source[key]
    return replace(profile, profile_id=profile.profile_id+"-ready", parameters=parameters)


def move_to_ready_pose(driver, profile, targets, *, cancel=None):
    """Shared startup/in-session positioning; the caller owns the connection."""
    def check_cancelled():
        if cancel is not None and cancel.is_set():
            raise RuntimeError("机械臂回位已取消")

    check_cancelled()
    print_message("正在清理机械臂故障……")
    driver.clear_errors(tuple(targets), cancel=cancel)
    check_cancelled()
    driver.configure(profile)
    for side, target in targets.items():
        check_cancelled()
        label = "左臂" if side == "left" else "右臂"
        ratio = profile.parameters["arms"][side]["velocity_ratio"]
        print_message(f"{label}回位中 · 速度 {ratio}%")
        driver.move_joints(side, target, cancel=cancel, timeout_s=60.)
    check_cancelled()
