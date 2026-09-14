"""Return Tianji arms to the configured initial pose with --enable-motion."""

from bimanual_teleop.cli.prepare_tianji_teleop import parser, run


def main(argv=None):
    return run(parser().parse_args(argv))


if __name__ == "__main__":
    raise SystemExit(main())
