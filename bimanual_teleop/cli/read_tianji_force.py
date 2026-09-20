"""Read the wrist six-axis force sensors on one or both Tianji arms."""

import argparse
import sys
import time
from pathlib import Path

from bimanual_teleop.devices.tianji.config import DEFAULT_CONFIG, load_config
from bimanual_teleop.devices.tianji.sdk import ControlSDK, add_sdk_argument

# Feedback index, SDK arm ID, and DCSS_CMD_ARM{0,1}_GET_DATA_6FT.
ARMS = {"left": (0, "A", 116), "right": (1, "B", 216)}
RAW_SCALE = 10_000.0
SAMPLE_INTERVAL_S = 0.2
DATA_TIMEOUT_S = 3.0


def main(argv=None, *, default_side="both") -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--tianji-config", "--config", dest="config", type=Path, default=DEFAULT_CONFIG,
                        help="天机配置；默认 configs/tianji_teleop.yaml")
    parser.add_argument("--side", choices=("left", "right", "both"), default=default_side,
                        help=f"读取哪侧腕部六维力；默认 {default_side}")
    add_sdk_argument(parser)
    args = parser.parse_args(argv)
    sides = ("left", "right") if args.side == "both" else (args.side,)

    connected = False
    robot = None
    try:
        controller_ip = load_config(args.config)["controller_ip"]
        robot = ControlSDK(args.sdk_root)
        robot.open(controller_ip)
        connected = True
        for side in sides:
            _, arm, channel = ARMS[side]
            if not robot.robot.set_user_specified_data(arm, channel):
                raise RuntimeError(f"failed to select {side}-arm six-axis force data (channel {channel})")

        previous_frames = dict.fromkeys(sides)
        deadlines = dict.fromkeys(sides, time.monotonic() + DATA_TIMEOUT_S)
        print(f"持续读取腕部六维力（{args.side}）；按 Ctrl+C 退出。", file=sys.stderr, flush=True)
        while True:
            data = robot.read()
            now = time.monotonic()
            for side in sides:
                index, _, channel = ARMS[side]
                feedback = data.m_Out[index]
                frame = feedback.m_OutFrameSerial
                tag = feedback.m_EST_Joint_Firc[0]
                if frame == 0 or frame == previous_frames[side] or round(tag) != channel:
                    if now >= deadlines[side]:
                        raise RuntimeError(f"no fresh {side}-arm force data (frame={frame}, channel={tag})")
                    continue

                raw = feedback.m_EST_Joint_Firc_Dot[:6]
                values = [value / RAW_SCALE for value in raw]
                print(
                    f"{side} frame={frame}  "
                    f"F[N]=({values[0]:.4f}, {values[1]:.4f}, {values[2]:.4f})  "
                    f"T[N·m]=({values[3]:.4f}, {values[4]:.4f}, {values[5]:.4f})  "
                    f"raw={raw}",
                    flush=True,
                )
                previous_frames[side] = frame
                deadlines[side] = now + DATA_TIMEOUT_S
            time.sleep(SAMPLE_INTERVAL_S)
    except KeyboardInterrupt:
        return 0
    except (OSError, KeyError, RuntimeError, ValueError) as exc:
        print(f"读取腕部六维力失败：{exc}", file=sys.stderr)
        return 1
    finally:
        if connected:
            robot.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
