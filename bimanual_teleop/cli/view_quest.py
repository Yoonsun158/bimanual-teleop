"""Show live left/right Quest controller poses in an interactive 3D window."""

from __future__ import annotations

import argparse
import subprocess

from bimanual_teleop.common.console import configure_runtime_logging, print_message
from bimanual_teleop.devices.quest.adapter import QuestSource


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--serial", help="USB ADB serial; auto-selects a single Quest")
    args = parser.parse_args(argv)
    configure_runtime_logging()
    try:
        import matplotlib.pyplot as plt
        from bimanual_teleop.visualization.quest import QuestPoseView
    except ImportError as error:
        print_message(f"{error}；请使用 bimanual-teleop Conda 环境", "error")
        return 1

    source = QuestSource(serial=args.serial)
    view = QuestPoseView()
    timer = view.figure.canvas.new_timer(interval=33)
    timer.add_callback(lambda: view.update(source.get_latest(), source.health()))
    try:
        print_message("正在连接 Quest；关闭窗口退出。")
        source.start()
        timer.start()
        print_message("Quest 位姿显示已就绪。", "ready")
        plt.show()
    except KeyboardInterrupt:
        pass
    except (OSError, RuntimeError, ValueError, subprocess.SubprocessError) as error:
        print_message(f"Quest 显示失败：{error}", "error")
        return 1
    finally:
        timer.stop()
        source.close()
        plt.close(view.figure)
    print_message("已退出。", "done")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
