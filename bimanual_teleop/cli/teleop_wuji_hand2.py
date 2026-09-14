"""Read Wuji glove/Hand2 feedback, preview retargeting, or explicitly follow."""

from __future__ import annotations

import argparse
from pathlib import Path

from bimanual_teleop.common.console import configure_runtime_logging, print_message
from bimanual_teleop.paths import PROJECT_ROOT
from bimanual_teleop.cli.teleop_quest_tianji import HELP, TeleopUI, NonblockingTerminal, run_loop


ROOT = PROJECT_ROOT


def report_rates(status):
    parts = []
    for side, glove in status.get("gloves", {}).items():
        skeleton = glove["statistics"].get("skeleton", {})
        feedback = status["hands"][side]["statistics"].get("joints", {})
        parts.append(f"{'左' if side == 'left' else '右'}：骨架 {skeleton.get('observed_hz', 0):.1f} Hz，"
                     f"反馈 {feedback.get('observed_hz', 0):.1f} Hz，"
                     f"缺口 {skeleton.get('source_gaps', 0)}/{feedback.get('source_gaps', 0)}")
    if parts:
        rate = status.get("control_hz_actual")
        control = "等待样本" if rate is None else f"{rate:.1f} Hz"
        print_message(" · ".join(parts) + f" · 控制循环 {control}")


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--wuji-config", "--config", dest="config", type=Path,
                        default=ROOT / "configs/wuji_teleop.json",
                        help="Wuji 配置；默认 configs/wuji_teleop.json")
    parser.add_argument("--side", choices=("left", "right", "both"), default="both")
    user = parser.add_mutually_exclusive_group()
    user.add_argument("--user-name", help="按已有 SDK 用户名选择用户；默认从配置文件读取")
    user.add_argument("--user-id", "--sdk-user-id", dest="sdk_user_id",
                      help="兼容旧配置或区分同名用户的实际 SDK ID")
    parser.add_argument("--enable-motion", action="store_true", help="allow Enter to enable hand following")
    parser.add_argument("--verbose", action="store_true", help="显示详细运行状态和 SDK 信息")
    args = parser.parse_args(argv)
    runtime = ui = None
    timing = error = None
    try:
        configure_runtime_logging(verbose=args.verbose, wuji=True)
        from bimanual_teleop.control.hand.follow import create_wuji_teleop, load_config, preflight
        config = load_config(args.config)
        if args.user_name is not None:
            config.pop("sdk_user_id", None)
            config["sdk_user_name"] = args.user_name
        elif args.sdk_user_id is not None:
            config.pop("sdk_user_name", None)
            config["sdk_user_id"] = args.sdk_user_id
        preflight()
        sides = ("left", "right") if args.side == "both" else (args.side,)
        with NonblockingTerminal() as terminal:
            runtime = create_wuji_teleop(config, sides=sides, sink=None, enable_motion=args.enable_motion)
            print_message(("手部遥操作" if args.enable_motion else "手部只读预览") + "\n" + HELP)
            runtime.start()
            ui = TeleopUI(runtime, None, enable_motion=args.enable_motion, background_engage=True,
                            verbose=args.verbose,
                            following_message="已接合，灵巧手正在跟随手套。")
            timing = run_loop(runtime, ui, terminal, period_ns=round(1e9 / config.get("control_hz", 120)),
                              status_reporter=report_rates if args.verbose else None)
    except KeyboardInterrupt:
        pass
    except (OSError, RuntimeError, ValueError, TypeError, ImportError) as problem:
        error = str(problem)
    finally:
        # Finish any cancelled activation before releasing its SDK objects.
        for component in (ui, runtime):
            if component is not None:
                try:
                    component.close()
                except (OSError, RuntimeError, ValueError) as problem:
                    error = f"{error + '; ' if error else ''}{problem}"
    if error:
        print_message(error, "error")
    else:
        print_message("已退出。" if timing is None else f"已退出，运行 {timing['elapsed_s']:.1f} 秒。", "done")
    return 1 if error else 0


if __name__ == "__main__":
    raise SystemExit(main())
