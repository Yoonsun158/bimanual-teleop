"""Return Tianji arms to the configured initial pose after an Enter confirmation."""

from bimanual_teleop.cli.prepare_tianji_teleop import parser, run


def main(argv=None):
    return run(parser().parse_args(argv))


if __name__ == "__main__":
    raise SystemExit(main())
