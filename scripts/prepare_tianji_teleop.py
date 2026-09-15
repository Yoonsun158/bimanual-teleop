"""Inspect Tianji joints, or confirm with Enter to prepare the initial pose."""

from bimanual_teleop.cli.prepare_tianji_teleop import parser, run


def main(argv=None):
    return run(parser().parse_args(argv))


if __name__ == "__main__":
    raise SystemExit(main())
