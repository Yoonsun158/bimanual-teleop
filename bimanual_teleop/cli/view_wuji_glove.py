"""Show one Wuji Glove's hand skeleton, contact locations and pressure."""

from __future__ import annotations

import argparse

from bimanual_teleop.common.console import configure_runtime_logging, print_message
from bimanual_teleop.devices.wuji.config import add_glove_arguments, glove_settings
from bimanual_teleop.devices.wuji.adapter import WujiGloveSource, WujiSdkSession


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    add_glove_arguments(parser)
    parser.add_argument("--verbose", action="store_true", help="显示详细设备状态和 SDK 信息")
    args = parser.parse_args(argv)
    source = session = view = timer = None
    error = None
    try:
        configure_runtime_logging(verbose=args.verbose, wuji=True)
        import matplotlib.pyplot as plt
        from bimanual_teleop.visualization.wuji import WujiGloveView

        address, user = glove_settings(args.config, args.side, args.address, args.sdk_user_id,
                                      user_name=args.user_name)
        print_message(f"正在连接{('左' if args.side == 'left' else '右')}手套。")
        session = WujiSdkSession(**user).open()
        source = WujiGloveSource(args.side, address, manager=session.manager, sdk=session.sdk,
                                 streams=("skeleton", "tactile", "contact"))
        source.start()
        if args.verbose:
            print_message(f"SDK 用户：{session.metadata.get('sdk_user_name')} ({session.metadata.get('sdk_user_id')})；"
                          f"模型：{source.metadata.get('human_model', 'unknown')}")
        view = WujiGloveView(args.side)
        timer = view.figure.canvas.new_timer(interval=50)
        timer.add_callback(lambda: view.update(source, session=session))
        timer.start()
        print_message("手套骨架与触觉显示已就绪；颜色表示相对压力，黑框表示 SDK 检测到接触；关闭窗口退出。", "ready")
        if not source.metadata.get("tactile_contact_model_present"):
            print_message("当前用户和手套缺少触觉接触模型；仅显示压力，接触状态未知。"
                          "请用 calibrate_wuji_glove.py --kind tactile 完成标定。", "warning")
        plt.show()
    except KeyboardInterrupt:
        pass
    except (OSError, RuntimeError, ValueError, KeyError, TypeError, ImportError) as problem:
        error = str(problem)
    finally:
        if timer is not None:
            timer.stop()
        if source is not None:
            try:
                source.close()
            except Exception as problem:
                error = f"{error}; {problem}" if error else str(problem)
        if session is not None:
            session.close()
        if view is not None:
            plt.close(view.figure)
    if error:
        print_message(error, "error")
    else:
        print_message("已退出。", "done")
    return 1 if error else 0


if __name__ == "__main__":
    raise SystemExit(main())
