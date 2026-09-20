"""Compatibility entry point; read the right wrist force sensor by default."""

from bimanual_teleop.cli.read_tianji_force import main as read_force


def main(argv=None) -> int:
    return read_force(argv, default_side="right")


if __name__ == "__main__":
    raise SystemExit(main())
