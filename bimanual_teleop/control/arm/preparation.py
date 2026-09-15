"""Finish cancellable initial positioning before another controller connects."""

from pathlib import Path
import signal
import subprocess
import sys

from bimanual_teleop.common.console import print_message
from bimanual_teleop.paths import PROJECT_ROOT


ROOT = PROJECT_ROOT


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


def prepare_initial_pose(*, config, ip, terminal, side="both", library=None, model=None):
    """Run after the caller has received the operator's motion confirmation."""
    config = Path(config).resolve()
    worker = ("from bimanual_teleop.cli.prepare_tianji_teleop import parser, run; "
              "raise SystemExit(run(parser().parse_args(), confirmed=True))")
    command = [sys.executable, "-c", worker,
               "--ip", ip, "--tianji-config", str(config), "--side", side]
    for flag, path in (("--library", library), ("--model", model)):
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
