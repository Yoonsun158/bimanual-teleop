"""Run the SDK's guided joint or tactile calibration for a named Wuji user."""

from __future__ import annotations

import argparse

from bimanual_teleop.common.console import StatusConsole, configure_runtime_logging, print_message
from bimanual_teleop.devices.wuji.config import add_glove_arguments, glove_settings
from bimanual_teleop.calibration.wuji import calibrate_glove


JOINT_POSES = {
    "pinch_index": "拇指和食指仅用指尖轻触，不要压住指腹；保持不动。",
    "pinch_middle": "拇指和中指仅用指尖轻触，不要压住指腹；保持不动。",
    "pinch_ring": "拇指和无名指仅用指尖轻触，不要压住指腹；保持不动。",
    "pinch_pinky": "拇指和小指仅用指尖轻触，不要压住指腹；保持不动。",
    "four_finger_bend_90": "食指到小指一起弯曲约 90°，像半握拳；拇指放松，不要握紧。",
    "flat_open": "食指到小指伸直并靠拢；拇指放松，手掌保持稳定。",
}

TACTILE_MOTIONS = {
    "four_finger_L_thumb_in": "四指在根部关节向下弯约 90°再伸直，反复；最后一次弯曲时拇指向掌心收。",
    "claw_curl": "五指向掌心弯成爪形，再张开；持续重复。",
    "abduction": "五指伸直，尽量张开，再靠拢；持续重复，手指不要相碰。",
    "finger_to_palm": "从食指到小指依次向掌心弯，再伸开；指尖不要碰到掌心或其他手指。",
}

FINGER_NAMES = {
    "thumb": "拇指", "index": "食指", "middle": "中指",
    "ring": "无名指", "pinky": "小指", "little": "小指",
}


def calibration_hint(hint):
    directions = {"increase": "增大", "decrease": "减小"}
    words = hint.lower().split() if isinstance(hint, str) else []
    if len(words) == 4 and words[:2] == ["rotate", "to"] and words[2] in directions:
        return f"使 {words[3]} {directions[words[2]]}"
    return hint


def is_extreme_yaw_metric(metric):
    return (isinstance(metric, dict)
            and str(metric.get("label", "")).lower() == "yaw"
            and isinstance(metric.get("value"), (int, float))
            and abs(metric["value"]) >= 150
            and isinstance(metric.get("min"), (int, float))
            and isinstance(metric.get("max"), (int, float))
            and metric["min"] < 0 < metric["max"])


def metric_diagnostic(metric):
    """Keep SDK pose diagnostics attached to the finger they describe."""
    if not isinstance(metric, dict):
        return None
    value = metric.get("value")
    minimum, maximum = metric.get("min"), metric.get("max")
    bounds = [bound for bound in (minimum, maximum) if isinstance(bound, (int, float))]
    if isinstance(value, (int, float)) and bounds:
        if (not isinstance(minimum, (int, float)) or value >= minimum) and (not isinstance(maximum, (int, float)) or value <= maximum):
            return None
    elif isinstance(metric.get("error"), (int, float)) and metric["error"] <= 0:
        return None

    finger = metric.get("finger")
    finger_b = metric.get("finger_b")
    fingers = [FINGER_NAMES.get(name.lower(), name) for name in (finger, finger_b)
               if isinstance(name, str) and name]
    label = metric.get("label") or "角度"
    unit = {"deg": "°", "degree": "°"}.get(metric.get("unit"), metric.get("unit") or "")
    parts = ["-".join(fingers) + f" {label}" if fingers else str(label)]
    if isinstance(value, (int, float)):
        parts.append(f"当前 {value:g}{unit}")
    if bounds:
        low = f"{minimum:g}" if isinstance(minimum, (int, float)) else "-∞"
        high = f"{maximum:g}" if isinstance(maximum, (int, float)) else "+∞"
        parts.append(f"目标 {low}～{high}{unit}")
    hint = metric.get("hint")
    if hint and not is_extreme_yaw_metric(metric):
        parts.append(calibration_hint(hint))
    return "；".join(parts)


def extreme_yaw_count(metrics):
    """Count fingers whose yaw is far outside a near-zero target interval."""
    return sum(is_extreme_yaw_metric(metric) for metric in metrics)

STATES = {
    "waiting_movement": "先完全张开手，再摆出本步姿势；系统会自动识别。",
    "waiting_stable": "姿势已识别；保持手腕和手指不动。",
    "holding": "保持当前姿势，手腕和手指不要移动。",
    "stabilizing": "正在检查姿势稳定性；保持不动。",
    "collecting": "正在采集；继续保持当前姿势。",
    "collect": "正在采集；持续重复本步动作。",
    "train": "正在训练触觉模型。",
    "check": "正在检查采集数据。",
    "install": "正在安装触觉模型。",
    "verify": "正在验证模型加载。",
}


def step_label(info):
    index, total = info.get("step_index"), info.get("step_total")
    return f"[{index + 1}/{total}]" if isinstance(index, int) and isinstance(total, int) and total > 0 else ""


class CalibrationGuide:
    def __init__(self, kind):
        self.kind = kind
        self.status = StatusConsole()
        self.last_step = None

    def feedback(self, info):
        name = info.get("step_name", "")
        label = step_label(info)
        step = (info.get("step_index"), name)
        if self.kind == "joints" and name and step != self.last_step:
            self.last_step = step
            print_message(f"{label} {JOINT_POSES.get(name, name)}".strip(), "ready")

        state = info.get("state", "")
        if state:
            if state == "done":
                index, total = info.get("step_index"), info.get("step_total")
                more = isinstance(index, int) and isinstance(total, int) and index + 1 < total
                if self.kind == "joints":
                    detail = "本步完成；现在完全张开手，等待下一步。" if more else "六步采集完成；等待模型生成。"
                else:
                    detail = "本步完成。"
            elif state == "waiting_stable" and info.get("constraints_ok") is False:
                detail = "检测到当前动作；姿势尚未达标，调整后保持手腕和手指不动。"
            else:
                detail = STATES.get(state, state)
            if state == "collect" and isinstance(info.get("collect_elapsed"), (int, float)) and isinstance(info.get("collect_target"), (int, float)):
                elapsed, target = info["collect_elapsed"], info["collect_target"]
                detail += f" {min(int(elapsed // 5) * 5, target):g}/{target:g} 秒"
            elif state == "train" and isinstance(info.get("epoch"), int) and isinstance(info.get("epoch_total"), int):
                epoch = info["epoch"] // 10 * 10
                if epoch:
                    detail += f" {epoch}/{info['epoch_total']} 轮"
            elif state in ("waiting_stable", "collecting") and isinstance(info.get("progress"), (int, float)):
                progress = min(100, max(0, int(info["progress"] * 100 // 25) * 25))
                if progress:
                    detail += f" {progress}%"
            self.status.state(f"{label} {detail}".strip())

        if info.get("variance_ok") is False:
            self.status.warning("手部移动过大；保持手腕和手指稳定。", key=(step, "variance"))
        metrics = info.get("metrics") or []
        if info.get("constraints_ok") is False:
            if name == "four_finger_bend_90":
                if extreme_yaw_count(metrics) >= 2:
                    message = ("多指 Yaw 接近 ±180°；若持续不变，先检查手套固件和"
                               "指尖 EMF 模块方向，不要继续硬调手指。")
                else:
                    message = "四指角度未达标；保持手腕稳定，逐根调整弯曲和张开幅度。"
            else:
                message = "姿势未满足要求；按本步动作调整后保持。"
            self.status.warning(message, key=(step, "constraints"))
        diagnostics = [line for metric in metrics
                       if (line := metric_diagnostic(metric))]
        if info.get("constraints_ok") is False and diagnostics:
            for line in diagnostics[:8]:
                self.status.warning(f"未通过：{line}", key=(step, line.split("；", 1)[0]))
        else:
            for hint in info.get("hints") or []:
                self.status.warning(f"SDK 提示：{calibration_hint(hint)}", key=(step, hint))

    def pose_prompt(self, info):
        kind = info.get("kind")
        if kind not in ("pose_ready", "pose_review"):
            return "proceed"
        label = step_label(info)
        if kind == "pose_ready":
            name = info.get("step_name", "")
            print_message(f"{label} {TACTILE_MOTIONS.get(name, name)}".strip(), "ready")
            seconds = info.get("seconds_per_pose")
            duration = f"约 {seconds:g} 秒" if isinstance(seconds, (int, float)) and seconds > 0 else "直至采集结束"
            print_message(f"空手，不碰掌心、其他手指或物体；回车后持续做本步动作{duration}。")
            question = "回车开始采集，q 退出："
        else:
            print_message(f"{label} 采集结束。".strip())
            for warning in info.get("warnings") or []:
                print_message(f"采集质量：{warning}", "warning")
            question = "回车保留并继续，r 重录本步，q 退出："
        while True:
            try:
                answer = input(question).strip().lower()
            except (EOFError, KeyboardInterrupt):
                return "abort"
            if answer == "":
                return "proceed"
            if answer == "q":
                return "abort"
            if answer == "r" and kind == "pose_review":
                return "retry"
            print_message("请输入回车或提示中的选项。", "warning")


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    add_glove_arguments(parser, include_sdk_user=False)
    parser.add_argument("--kind", choices=("joints", "tactile"), required=True)
    parser.add_argument("--user-name", required=True, help="按姓名选择用户；没有同名用户时创建")
    args = parser.parse_args(argv)
    try:
        configure_runtime_logging(wuji=True)
        address, _ = glove_settings(args.config, args.side, args.address, user_name=args.user_name)
        guide = CalibrationGuide(args.kind)
        print_message(f"开始{('关节' if args.kind == 'joints' else '触觉')}标定；设备 {args.side}。")
        if args.kind == "joints":
            print_message("共 6 个静态姿势；每步摆好后保持不动，SDK 自动采集，无需按键；步间完全张开手。")
        else:
            print_message("共 4 个无接触动作；每步回车开始采集，动作期间持续重复；采集结束可保留或重录。")
        result = calibrate_glove(args.side, address, kind=args.kind,
                                 user_name=args.user_name, on_feedback=guide.feedback,
                                 on_pose_prompt=guide.pose_prompt if args.kind == "tactile" else None)
        user = result["sdk_user"]
        if args.kind == "joints":
            print_message(f"关节标定完成；用户 {user.get('display_name', '')}；"
                          f"手模型 {result.get('model', 'unknown')}。", "done")
        else:
            summary = result.get("result", {})
            model = summary.get("model_dir", "未提供路径") if isinstance(summary, dict) else "未提供路径"
            print_message(f"触觉标定完成；用户 {user.get('display_name', '')}；"
                          f"触觉模型 {model}。", "done")
        print_message(f"后续显示或控制可在 configs/wuji_teleop.yaml 中设置 "
                      f"sdk_user_name={user['display_name']!r}，或使用 --user-name {user['display_name']!r}")
        return 0
    except KeyboardInterrupt:
        print_message("标定已取消。", "warning")
        return 130
    except (OSError, RuntimeError, ValueError, KeyError, TypeError, ImportError, EOFError) as error:
        print_message(str(error), "error")
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
