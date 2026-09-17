"""Clear Tianji controller and servo errors without enabling or moving arms."""

import argparse
from pathlib import Path

from bimanual_teleop.devices.tianji.sdk import add_sdk_argument
from bimanual_teleop.common.console import print_message
from bimanual_teleop.control.arm.preparation import await_feedback
from bimanual_teleop.devices.tianji.config import DEFAULT_CONFIG, load_config
from bimanual_teleop.devices.tianji.driver import TianjiDriver


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--tianji-config", "--config", dest="config", type=Path, default=DEFAULT_CONFIG)
    parser.add_argument("--ip", help="临时覆盖天机控制器 IP")
    add_sdk_argument(parser)
    parser.add_argument("--side", choices=("left", "right", "both"), default="both")
    args = parser.parse_args(argv)
    driver = None
    error = None
    try:
        settings = load_config(args.config, args.ip)
        print_message("请确保机械臂已停止、实体急停已释放；本命令只清错，不使能或运动。")
        driver = TianjiDriver(settings["controller_ip"], args.sdk_root)
        driver.start()
        await_feedback(driver)
        sides = ("left", "right") if args.side == "both" else (args.side,)
        result = driver.clear_errors(sides)
        print_message("故障已清除。" if result["requested_arms"] else "所选机械臂无故障。", "done")
    except (OSError, RuntimeError, ValueError, KeyboardInterrupt) as problem:
        error = str(problem) or "清错已取消"
    finally:
        if driver is not None:
            try:
                driver.close()
            except (OSError, RuntimeError) as problem:
                error = error or str(problem)
    if error:
        print_message(error, "error")
    return int(error is not None)


if __name__ == "__main__":
    raise SystemExit(main())
