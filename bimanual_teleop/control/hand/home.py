"""Home either Wuji Hand2, or both in left-then-right order."""

from __future__ import annotations

import argparse

from bimanual_teleop.common.config import positive_number
import math
from pathlib import Path
import time

from bimanual_teleop.common.terminal import NonblockingTerminal, confirm_motion
from bimanual_teleop.common.console import configure_runtime_logging, print_message
from bimanual_teleop.devices.wuji.adapter import JOINT_LIMITS_RAD, JOINT_NAMES, WujiHandDriver
from bimanual_teleop.devices.wuji.config import DEFAULT_CONFIG
from bimanual_teleop.devices.wuji.config import load_config
from bimanual_teleop.types import ControlProfile, DeviceCommand, JointTarget


CONTROL_HZ = 120
PERIOD_NS = round(1e9 / CONTROL_HZ)
COMMAND_TTL_NS = 50_000_000


def home_target(start_rad: tuple[float, ...], elapsed_s: float,
                duration_s: float = 3.) -> JointTarget:
    """Smoothstep from measured position to the Hand2-defined joint zero."""
    positive_number(duration_s, "duration_s")
    if len(start_rad) != len(JOINT_NAMES) or any(not math.isfinite(q) for q in start_rad):
        raise ValueError("Hand2 needs 20 finite measured joint angles")
    if any(not low <= q <= high for q, (low, high) in zip(start_rad, JOINT_LIMITS_RAD)):
        raise ValueError("measured Hand2 joints exceed mechanical limits")
    alpha = max(0., min(1., elapsed_s / duration_s))
    alpha = alpha*alpha*(3 - 2*alpha)
    return JointTarget(JOINT_NAMES, tuple((1-alpha)*q for q in start_rad))


def feedback_error_rad(sample, *, after_ns: int) -> float | None:
    """No fabricated zero for missing or older feedback."""
    if sample is None or not sample.header.valid or sample.header.received_monotonic_ns < after_ns:
        return None
    joints = sample.payload.position_rad
    if len(joints) != len(JOINT_NAMES) or any(q is None or not math.isfinite(q) for q in joints):
        return None
    return max(abs(q) for q in joints)


def run_home(hand: WujiHandDriver, profile: ControlProfile, *, duration_s: float = 3.,
             tolerance_rad: float = math.radians(5), settle_timeout_s: float = 2.,
             hold_at_zero: bool = True, terminal=None, emit=None) -> bool:
    """Return True on arrival without holding, False on Q/EOF; always disable."""
    positive_number(duration_s, "duration_s")
    positive_number(tolerance_rad, "tolerance_rad")
    positive_number(settle_timeout_s, "settle_timeout_s")
    terminal = terminal or NonblockingTerminal()
    say = emit or print_message
    try:
        with terminal as keys:
            hand.configure(profile)
            hand.engage()
            if hand.last_target is None:
                raise RuntimeError("Hand2 did not retain its measured engagement target")
            start_rad = tuple(hand.last_target.position_rad)
            home_target(start_rad, 0, duration_s)
            start_ns = time.monotonic_ns()
            motion_end_ns = start_ns + round(duration_s * 1e9)
            settle_end_ns = motion_end_ns + round(settle_timeout_s * 1e9)
            deadline = start_ns
            count = 0
            arrived = False
            say(f"Hand2 从实测位置平滑回到 20 个零角，约 {duration_s:g} 秒；按 Q 中止/退出。")
            while True:
                now = time.monotonic_ns()
                if now < deadline:
                    time.sleep((deadline - now) / 1e9)
                    now = time.monotonic_ns()
                deadline += PERIOD_NS
                if deadline <= now:
                    deadline = now + PERIOD_NS
                key_batch = keys.read(0)
                if key_batch == "" or (key_batch and "q" in key_batch.lower()):
                    return False
                health = hand.health(check_latch=True)
                if not health.ready:
                    raise RuntimeError(f"Hand2 feedback/diagnostics failed: {health.detail}")
                sample = hand.get_latest()
                if sample is None or not sample.header.valid or (
                        now - sample.header.received_monotonic_ns > hand.timeout_ns):
                    raise RuntimeError("Hand2 joint feedback invalid or stale")
                target = home_target(start_rad, (now - start_ns) / 1e9, duration_s)
                count += 1
                command = DeviceCommand(
                    hand.device_id, f"{hand.device_id}-home-{count}", target, (sample.header.ref,),
                    now, now + COMMAND_TTL_NS, profile.profile_id)
                result = hand.submit(command)
                if not result.accepted:
                    raise RuntimeError(f"Hand2 home command rejected: {result.reason}")
                if now >= motion_end_ns and not arrived:
                    error = feedback_error_rad(hand.get_latest(), after_ns=motion_end_ns)
                    if error is not None and error <= tolerance_rad:
                        arrived = True
                        detail = "持续保持零角，按 Q 退出。" if hold_at_zero else "本侧回零完成。"
                        say(f"已到位，最大反馈误差 {math.degrees(error):.1f}°；{detail}")
                        if not hold_at_zero:
                            return True
                    elif now >= settle_end_ns:
                        detail = "无有效末端反馈" if error is None else f"最大误差 {math.degrees(error):.1f}°"
                        raise RuntimeError(f"Hand2 回零未确认：{detail}")
    finally:
        hand.disable("Hand2 home exited")


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--side", choices=("left", "right", "both"), required=True,
                        help="选择左手、右手；both 按先左后右顺序回零")
    parser.add_argument("--wuji-config", "--config", dest="config", type=Path, default=DEFAULT_CONFIG,
                        help="Wuji 配置；默认 configs/wuji_teleop.yaml")
    parser.add_argument("--duration-s", type=float, default=3., help="每只手平滑回零时间，默认 3 秒")
    parser.add_argument("--tolerance-deg", type=float, default=5., help="到位反馈容差，默认 5°")
    parser.add_argument("--settle-timeout-s", type=float, default=2., help="末端反馈等待时间，默认 2 秒")
    args = parser.parse_args(argv)
    error = None
    try:
        configure_runtime_logging(wuji=True)
        duration_s = positive_number(args.duration_s, "--duration-s")
        tolerance_rad = math.radians(positive_number(args.tolerance_deg, "--tolerance-deg"))
        settle_timeout_s = positive_number(args.settle_timeout_s, "--settle-timeout-s")
        config = load_config(args.config)
        profile = ControlProfile(config["profile_id"], config.get("mode", "mit"), config["parameters"])
        sides = ("left", "right") if args.side == "both" else (args.side,)
        with NonblockingTerminal() as terminal:
            if not confirm_motion(terminal, "开始灵巧手回零"):
                return 0
        for side in sides:
            hand = WujiHandDriver(side, config["devices"][side]["hand"],
                                  timeout_s=config.get("hand_timeout_s", .5))
            try:
                hand.start()
                deadline = time.monotonic() + 3
                while not hand.health(check_latch=True).ready and time.monotonic() < deadline:
                    time.sleep(.02)
                health = hand.health(check_latch=True)
                if not health.ready:
                    raise RuntimeError(f"{side} Hand2: {health.detail}")
                sample = hand.get_latest()
                if sample is None:
                    raise RuntimeError(f"{side} Hand2 feedback unavailable")
                start = tuple(sample.payload.position_rad)
                home_target(start, 0, duration_s)
                print_message(f"{side} Hand2 当前最大零位偏差 {math.degrees(max(abs(q) for q in start)):.1f}°；"
                              f"硬件版本 {hand.metadata.get('hardware_version')}")
                if not run_home(hand, profile, duration_s=duration_s, tolerance_rad=tolerance_rad,
                                settle_timeout_s=settle_timeout_s, hold_at_zero=args.side != "both"):
                    break
            finally:
                hand.close()
    except KeyboardInterrupt:
        pass
    except Exception as problem:
        error = str(problem)
    if error:
        print_message(f"Hand2 回零：{error}", "error")
    else:
        print_message("已退出。", "done")
    return 1 if error else 0


if __name__ == "__main__":
    raise SystemExit(main())
