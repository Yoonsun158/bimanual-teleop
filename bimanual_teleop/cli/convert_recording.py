"""将原始遥操作记录对齐到主相机时间戳并导出 Diffusion Policy 数据集。"""

import argparse
from pathlib import Path
import sys

from bimanual_teleop.recording.convert import convert_recordings


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--input", required=True, type=Path, help="原始记录根目录、session 或单个 episode")
    parser.add_argument("--output", required=True, type=Path, help="新建的 .zarr 数据集，禁止覆盖")
    parser.add_argument("--action-space", required=True, choices=("eef", "joint"),
                        help="eef: 双臂目标末端+双手（52维）；joint: 双臂目标关节+双手（54维）")
    args = parser.parse_args(argv)
    try:
        report = convert_recordings(args.input, args.output, action_space=args.action_space)
    except (OSError, ValueError, KeyError, ImportError, RuntimeError) as error:
        print(f"转换失败：{error}", file=sys.stderr)
        return 1
    print(f"已导出 {report['output_episodes']} 个连续片段、{report['output_frames']} 帧：{args.output}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
