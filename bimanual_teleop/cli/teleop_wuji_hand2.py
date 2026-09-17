"""Control Hand2 from Wuji gloves after confirming the surroundings are safe."""

from __future__ import annotations

import argparse
from pathlib import Path

from bimanual_teleop.common.console import configure_runtime_logging, print_message, runtime_message
from bimanual_teleop.common.terminal import NonblockingTerminal, confirm_motion
from bimanual_teleop.devices.wuji.config import DEFAULT_CONFIG, sdk_user_name
from bimanual_teleop.cli.runtime import HELP, TeleopUI, run_loop


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--wuji-config", "--config", dest="config", type=Path,
                        default=DEFAULT_CONFIG,
                        help="Wuji 配置；默认 configs/wuji_teleop.yaml")
    parser.add_argument("--side", choices=("left", "right", "both"), default="both")
    parser.add_argument("--user-name", help="按已有 SDK 用户名选择用户；默认从配置文件读取")
    parser.add_argument("--viewer", action="store_true", help="显示已连接 RealSense 的彩色图像；默认关闭")
    parser.add_argument("-v", "--verbose", action="store_true",
                        help="输出 SDK 告警、调试日志及完整诊断；默认只显示关键状态和故障")
    args = parser.parse_args(argv)
    runtime = ui = None
    preview = None
    timing = error = None
    try:
        configure_runtime_logging(wuji=True, verbose=args.verbose)
        from bimanual_teleop.control.hand.follow import create_wuji_teleop, preflight
        from bimanual_teleop.devices.wuji.config import load_config
        config = load_config(args.config)
        if args.user_name is not None:
            config["sdk_user_name"] = sdk_user_name({}, user_name=args.user_name)
        preflight()
        sides = ("left", "right") if args.side == "both" else (args.side,)
        with NonblockingTerminal() as terminal:
            if not confirm_motion(terminal, "启动手部遥操作，设备就绪后等待接合"):
                return 0
            if args.viewer:
                from bimanual_teleop.visualization.realsense import RealSensePreview
                preview = RealSensePreview()
                preview.start()
            runtime = create_wuji_teleop(config, sides=sides, sink=None)
            print_message("手部遥操作\n" + HELP)
            runtime.start()
            ui = TeleopUI(runtime, None, background_engage=True,
                            verbose=args.verbose,
                            following_message="已接合，灵巧手正在跟随手套。")
            timing = run_loop(runtime, ui, terminal, period_ns=round(1e9 / config.get("control_hz", 120)))
    except KeyboardInterrupt:
        pass
    except (OSError, RuntimeError, ValueError, TypeError, ImportError) as problem:
        error = str(problem)
    finally:
        component = ui if ui is not None else runtime
        if component is not None:
            try:
                component.close()
            except (OSError, RuntimeError, ValueError) as problem:
                error = f"{error + '; ' if error else ''}{problem}"
        if preview is not None:
            preview.close()
    if error:
        print_message(runtime_message(error, verbose=args.verbose), "error")
    else:
        print_message("已退出。" if timing is None else f"已退出，运行 {timing['elapsed_s']:.1f} 秒。", "done")
    return 1 if error else 0


if __name__ == "__main__":
    raise SystemExit(main())
