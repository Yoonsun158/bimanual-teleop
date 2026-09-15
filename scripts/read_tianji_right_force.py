"""Read the six-axis force sensor installed on the Tianji right arm."""

import argparse
import sys
import time
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from bimanual_teleop.devices.tianji.config import DEFAULT_CONFIG, load_config

CHANNEL_RIGHT_6FT = 216
RAW_SCALE = 10_000.0
SAMPLE_INTERVAL_S = 0.2


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--tianji-config", "--config", dest="config", type=Path, default=DEFAULT_CONFIG,
                        help="天机配置；默认 configs/tianji_teleop.yaml")
    parser.add_argument("--sdk-root", type=Path, default=ROOT.parent / "TJ_FX_ROBOT_CONTRL_SDK")
    args = parser.parse_args(argv)

    controller_ip = load_config(args.config)["controller_ip"]
    if not (args.sdk_root / "SDK_PYTHON/fx_robot.py").is_file():
        parser.error(f"Tianji Python SDK not found under {args.sdk_root}")
    sys.path.insert(0, str(args.sdk_root))
    from SDK_PYTHON.fx_robot import DCSS, Marvin_Robot

    robot = Marvin_Robot()
    connected = False
    try:
        connected = bool(robot.connect(controller_ip))
        if not connected:
            raise RuntimeError("controller connection failed (UDP port 4730 may be in use)")
        if not robot.set_user_specified_data("B", CHANNEL_RIGHT_6FT):
            raise RuntimeError("failed to select right-arm six-axis force data (channel 216)")

        dcss = DCSS()
        previous_frame = None
        deadline = time.monotonic() + 3.0
        print("持续读取右臂六维力；按 Ctrl+C 退出。", file=sys.stderr, flush=True)
        while True:
            data = robot.subscribe(dcss)
            right = data["outputs"][1]
            frame = right["frame_serial"]
            tag = right["est_joint_firc"][0]
            if frame == 0 or frame == previous_frame or round(tag) != CHANNEL_RIGHT_6FT:
                if time.monotonic() >= deadline:
                    raise RuntimeError(f"no fresh right-arm force data (frame={frame}, channel={tag})")
                time.sleep(0.01)
                continue

            raw = right["est_joint_firc_dot"][:6]
            values = [value / RAW_SCALE for value in raw]
            print(
                f"frame={frame}  "
                f"F[N]=({values[0]:.4f}, {values[1]:.4f}, {values[2]:.4f})  "
                f"T[N·m]=({values[3]:.4f}, {values[4]:.4f}, {values[5]:.4f})  "
                f"raw={raw}",
                flush=True,
            )
            previous_frame = frame
            deadline = time.monotonic() + 3.0
            time.sleep(SAMPLE_INTERVAL_S)
    except KeyboardInterrupt:
        return 0
    except (OSError, KeyError, RuntimeError, ValueError) as exc:
        print(f"读取右臂六维力失败：{exc}", file=sys.stderr)
        return 1
    finally:
        if connected:
            robot.release_robot()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
