"""Quest 与 Wuji 双臂双手遥操作；--arms-only 仅控制机械臂。"""

from __future__ import annotations

import argparse
from dataclasses import asdict, replace
from pathlib import Path
import subprocess
import threading
import time

from bimanual_teleop.types import ControlProfile
from bimanual_teleop.common.console import configure_runtime_logging, print_message
from bimanual_teleop.common.terminal import NonblockingTerminal, confirm_motion
from bimanual_teleop.devices.tianji.config import DEFAULT_CONFIG, load_config
from bimanual_teleop.devices.wuji.config import DEFAULT_CONFIG as DEFAULT_WUJI_CONFIG
from bimanual_teleop.control.arm import preparation


PERIOD_NS = 5_000_000
HELP = "Enter 开始/恢复 · Space 暂停/取消 · Q 退出"
GESTURE_HELP = "Enter 或双手同时比 V 保持 0.3 秒：开始/恢复 · 任一手摇滚保持 0.3 秒：暂停 · Space 暂停/取消 · Q 退出"
MAPPING_HELP = "左手柄控制右臂，右手柄控制左臂；坐标系由配置 quest.coordinate_frame 选择，默认 headset。"


def select_profile_side(profile, side):
    """Make the selected arm the only native motion target."""
    if side == "both":
        return profile
    if side not in ("left", "right"):
        raise ValueError("side must be left, right, or both")
    return replace(profile, profile_id=f"{profile.profile_id}-{side}",
                   parameters={**profile.parameters, "active_arms": [side]})


def brief_reason(detail):
    detail = detail or "等待设备反馈"
    for prefix, message in (
        ("waiting for quest frames", "等待 Quest 数据"),
        ("quest left tracking unavailable", "左手柄未被追踪，请唤醒并放在头显可见范围"),
        ("quest right tracking unavailable", "右手柄未被追踪，请唤醒并放在头显可见范围"),
        ("quest xr session not focused", "请关闭 Quest 系统菜单，返回采集应用"),
        ("no tianji feedback", "等待机器人反馈"),
    ):
        if detail.lower().startswith(prefix):
            return message
    return detail


class TeleopUI:
    """Only UI state lives here; the runtime validates and executes each request."""

    def __init__(self, runtime, profile, *, enable_motion=False, emit=None,
                 background_engage=False, following_message="已接合，双臂正在跟随手柄。", gesture=None):
        self.runtime, self.profile = runtime, profile
        self.enable_motion = enable_motion
        self.emit = emit
        self.quit = False
        self.engage_pending = False
        self._last_error = None
        self.last_motion_error = None
        self.motion_pauses = 0
        self._last_status = None
        self._last_message = None
        self.background_engage = background_engage
        self.following_message = following_message
        self._engage_thread = None
        self._engage_error = None
        self._engage_cancelled = False
        self.gesture = gesture

    @property
    def start_hint(self):
        return "按 Enter 或双手重新比 V" if self.gesture is not None else "按 Enter"

    def say(self, message, level="info"):
        if message == self._last_message:
            return
        self._last_message = message
        if self.emit is None:
            print_message(message, level)
        else:
            self.emit(message)

    def report_status(self, status):
        health = status.get("health") or asdict(self.runtime.health())
        state = status.get("state", "").lower()
        current = (state, status.get("mode"), health["ready"],
                   health.get("detail") if not health["ready"] else None,
                   self.engage_pending)
        if current == self._last_status:
            return
        previous, self._last_status = self._last_status, current
        if not health["ready"]:
            detail = health.get("detail") or "等待设备反馈"
            if detail != self._last_error:
                self.say(self.waiting_message(detail), "warning")
        elif state != "engaged" and (previous is None or not previous[2]):
            self.say(f"设备已就绪，{self.start_hint}开始" +
                     ("遥操作。" if self.enable_motion else "只读预览。"), "ready")

    def waiting_message(self, detail):
        message = brief_reason(detail)
        return f"{message}；就绪后自动接合，Space 可取消。" if self.engage_pending else message

    def abort(self, reason):
        self.engage_pending = False
        self._engage_cancelled = True
        if self.gesture is not None:
            self.gesture.inhibit()
        self.runtime.pause(reason)
        if reason != self._last_error:
            self.say(brief_reason(reason), "warning")
        self._last_error = reason

    def handle(self, key):
        key = key.lower()
        try:
            if key in ("q", "\x04", "\x03"):
                self.engage_pending = False
                self._engage_cancelled = True
                self.runtime.pause("keyboard exit")
                self.quit = True
            elif key == " ":
                self.abort(f"键盘暂停；恢复须{self.start_hint}重新接合")
            elif key in ("\n", "\r"):
                self.request_engage(wait_until_ready=True)
        except (OSError, RuntimeError, ValueError) as error:
            self.last_motion_error = str(error)
            self.abort(str(error))

    def request_engage(self, *, wait_until_ready=False):
        state = getattr(self.runtime, "state", None)
        if (getattr(state, "value", state) == "engaged"
                or self._engage_thread is not None or self.quit):
            return False
        health = self.runtime.health()
        self.engage_pending = wait_until_ready and not health.ready
        if not health.ready:
            self.say(self.waiting_message(health.detail), "warning")
            return False
        if self.background_engage:
            self._engage_cancelled = False
            self._engage_error = None
            self._engage_thread = threading.Thread(target=self._engage, name="teleop-engage", daemon=True)
            self._engage_thread.start()
            self.say("正在接合；Space / Q 可取消。")
        else:
            self.runtime.engage(self.profile)
            self._engaged()
        return True

    def handle_gesture(self, command):
        if command == "pause":
            self.abort(f"摇滚手势暂停；{self.start_hint}恢复")
            return "pause"
        if command != "engage":
            raise ValueError(f"Unknown gesture command: {command}")
        state = self.runtime.state
        if getattr(state, "value", state) == "engaged" or self._engage_thread is not None:
            return "ignored"
        return "engage" if self.request_engage() else "not_ready"

    def _engaged(self):
        self._last_error = None
        self.say(self.following_message if self.enable_motion else "已开始只读预览。", "ready")

    def _engage(self):
        try:
            self.runtime.engage(self.profile)
        except Exception as error:
            self._engage_error = error

    def poll_engagement(self):
        thread = self._engage_thread
        if thread is None or thread.is_alive():
            return
        thread.join()
        self._engage_thread = None
        if self._engage_cancelled:
            self.runtime.pause("接合已取消")
        elif self._engage_error is not None:
            self.last_motion_error = str(self._engage_error)
            self.abort(self.last_motion_error)
        else:
            self._engaged()

    def close(self):
        if self._engage_thread is not None:
            self._engage_cancelled = True
            self.runtime.pause("关闭时取消接合")
            self._engage_thread.join(timeout=10.)
            if self._engage_thread.is_alive():
                raise RuntimeError("接合线程未在关闭期限内退出")
            self._engage_thread = None

    def report_runtime_pause(self):
        state = getattr(self.runtime, "state", None)
        reason = getattr(self.runtime, "last_error", None)
        if getattr(state, "value", state) == "paused" and reason and reason != self._last_error:
            if self.gesture is not None:
                self.gesture.inhibit()
            self.last_motion_error = self._last_error = reason
            self.motion_pauses += 1
            self.say(f"遥操作已暂停：{brief_reason(reason)}；恢复时{self.start_hint}。", "warning")


def prepare_initial_pose(args, terminal):
    """Finish the existing position runner before creating any teleop devices."""
    result = preparation.prepare_initial_pose(
        config=args.config, ip=args.robot_ip, terminal=terminal,
        side=getattr(args, "side", "both"), library=args.library,
        model=getattr(args, "model", None))
    print_message("正在连接 Quest。")
    return result


def create_runtime(args, profile, sink):
    from bimanual_teleop.devices.quest.adapter import QuestSource
    from bimanual_teleop.devices.tianji.driver import TianjiDriver
    from bimanual_teleop.devices.tianji.model import TianjiKinematics
    from bimanual_teleop.control.arm.quest import QuestTianjiTeleop
    from bimanual_teleop.control.arm.cartesian import TianjiCartesianExecutor

    quest = QuestSource(serial=args.serial)
    driver = TianjiDriver(args.robot_ip, args.library)
    kinematics = TianjiKinematics(args.library)
    executor = TianjiCartesianExecutor(driver, kinematics)
    arms = QuestTianjiTeleop(quest, driver, executor, kinematics, profile=profile,
                            sink=sink, enable_motion=True,
                            side=getattr(args, "side", "both"),
                            coordinate_frame=args.coordinate_frame)
    if args.arms_only:
        return arms
    from bimanual_teleop.control.hand.follow import create_wuji_teleop
    from bimanual_teleop.control.combined import QuestTianjiWujiTeleop
    hands = create_wuji_teleop(args.wuji_settings, sink=sink, enable_motion=True)
    return QuestTianjiWujiTeleop(arms, hands)


def run_loop(runtime, ui, terminal, *, period_ns=PERIOD_NS):
    next_tick = time.monotonic_ns()
    started = next_tick
    next_report = next_tick
    cycles = skipped = 0
    while not ui.quit:
        now = time.monotonic_ns()
        if now >= next_tick:
            missed = (now - next_tick) // period_ns
            skipped += missed
            # Rebase a late cycle instead of squeezing it next to the next one.
            next_tick = now + period_ns
            try:
                ui.poll_engagement()
                ui.report_runtime_pause()
                gesture = (ui.gesture.poll(start_ready=runtime.health().ready)
                           if ui.gesture is not None else None)
                if gesture is not None:
                    command, _sides = gesture
                    ui.handle_gesture(command)
                if ui.engage_pending and runtime.health().ready:
                    ui.handle("\n")
                runtime.tick(now)
            except (RuntimeError, ValueError) as error:
                ui.abort(str(error))
            ui.report_runtime_pause()
            cycles += 1
            finished = time.monotonic_ns()
            if finished >= next_tick:
                missed = (finished - next_tick) // period_ns + 1
                skipped += missed
                next_tick += missed * period_ns
        if now >= next_report:
            ui.report_status(runtime.status(include_target=False))
            next_report = now + 1_000_000_000
        keys = terminal.read(max(0, next_tick - time.monotonic_ns()) / 1e9)
        if keys == "":
            ui.handle("q")
        elif keys:
            for key in keys:
                ui.handle(key)
                if ui.quit:
                    break
    return {"cycles": cycles, "skipped_deadlines": skipped, "target_hz": round(1e9 / period_ns),
            "last_motion_error": ui.last_motion_error, "motion_pauses": ui.motion_pauses,
            "elapsed_s": (time.monotonic_ns() - started) / 1e9}


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__, epilog=MAPPING_HELP)
    parser.add_argument("--tianji-config", "--config", dest="config", type=Path, default=DEFAULT_CONFIG,
                        help="天机统一配置 YAML；默认 configs/tianji_teleop.yaml")
    parser.add_argument("--side", choices=("left", "right", "both"), default="both",
                        help="select the robot arm; left uses the right controller and vice versa")
    parser.add_argument("--serial", help="Quest USB adb serial")
    parser.add_argument("--robot-ip", help="临时覆盖设备配置中的天机控制器 IP")
    parser.add_argument("--library", type=Path, help="built libtianji_bridge.so")
    parser.add_argument("--wuji-config", type=Path, default=DEFAULT_WUJI_CONFIG,
                        help="Wuji 配置；默认 configs/wuji_teleop.yaml")
    parser.add_argument("--user-name", help="联合控制使用的已有 Wuji SDK 用户名；默认从配置文件读取")
    parser.add_argument("--arms-only", action="store_true", help="只控制机械臂，不连接 Wuji 手套和灵巧手")
    args = parser.parse_args(argv)
    if not args.arms_only and args.side != "both":
        parser.error("联合控制需要 --side both；单臂控制请加 --arms-only")
    combined = not args.arms_only
    runtime = ui = None
    error = None
    timing = None
    try:
        configure_runtime_logging(wuji=combined)
        settings = load_config(args.config, args.robot_ip)
        args.robot_ip = settings["controller_ip"]
        args.coordinate_frame = settings["quest"]["coordinate_frame"]
        profile = select_profile_side(
            ControlProfile(**settings["profile"]), args.side)
        if combined:
            from bimanual_teleop.control.hand.follow import load_config as load_wuji_config, preflight
            args.wuji_settings = load_wuji_config(args.wuji_config)
            if args.user_name is not None:
                from bimanual_teleop.devices.wuji.config import sdk_user_name
                args.wuji_settings["sdk_user_name"] = sdk_user_name({}, user_name=args.user_name)
            preflight()
        with NonblockingTerminal() as terminal:
            if not confirm_motion(terminal, "开始初始回位，完成后等待遥操作接合"):
                return 0
            prepare_initial_pose(args, terminal)
            runtime = create_runtime(args, profile, None)
            help_text = GESTURE_HELP if combined else HELP
            print_message("实机遥操作\n" + help_text)
            runtime.start()
            gesture = None
            if combined:
                from bimanual_teleop.control.hand.gesture import GestureCommands
                gesture = GestureCommands({side: glove.get_latest
                    for side, glove in runtime.hands.gloves.items()},
                    timeout_s=runtime.hands.glove_timeout_ns / 1e9)
            ui = TeleopUI(runtime, profile, enable_motion=True, gesture=gesture,
                            background_engage=combined,
                            following_message="已接合，双臂与双手正在跟随。" if combined else
                                              f"已接合，{'双臂' if args.side == 'both' else '左臂' if args.side == 'left' else '右臂'}正在跟随手柄。")
            timing = run_loop(runtime, ui, terminal)
    except KeyboardInterrupt:
        pass
    except (OSError, RuntimeError, ValueError, TypeError, ImportError, subprocess.SubprocessError) as problem:
        error = str(problem)
    finally:
        if ui is not None:
            try:
                ui.close()
            except (OSError, RuntimeError, ValueError) as problem:
                error = f"{error + '; ' if error else ''}取消接合失败：{problem}"
        if runtime is not None:
            try:
                runtime.close()
            except (OSError, RuntimeError, ValueError) as problem:
                error = f"{error + '; ' if error else ''}关闭运行模块失败：{problem}"
    if error:
        print_message(error, "error")
    else:
        summary = f"已退出，运行 {timing['elapsed_s']:.1f} 秒。" if timing else "已退出。"
        if timing and timing["motion_pauses"]:
            summary += f" 期间暂停 {timing['motion_pauses']} 次。"
        print_message(summary, "done")
    return 1 if error else 0


if __name__ == "__main__":
    raise SystemExit(main())
