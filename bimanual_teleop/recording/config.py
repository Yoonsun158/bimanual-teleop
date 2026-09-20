"""Small, fixed recording contract for this three-camera bimanual rig."""

from dataclasses import dataclass
from pathlib import Path
import math

from bimanual_teleop.common.config import load_yaml_config

DEFAULT_CONFIG = Path(__file__).resolve().parents[2] / "configs" / "recording.yaml"


@dataclass(frozen=True)
class RecordingConfig:
    cameras: tuple[str, str, str]
    main_depth: bool = True
    state_hz: float = 200.
    output_dir: str = "recordings"

    def __post_init__(self):
        if len(self.cameras) != 3 or len(set(self.cameras)) != 3 or any(
                not isinstance(s, str) or not s.strip() for s in self.cameras):
            raise ValueError("recording.cameras must contain three distinct serial strings")
        if type(self.main_depth) is not bool:
            raise ValueError("recording.main_depth must be true or false")
        if isinstance(self.state_hz, bool) or not math.isfinite(self.state_hz) or not 0 < self.state_hz <= 1000:
            raise ValueError("recording.state_hz must be in (0, 1000]")
        if not isinstance(self.output_dir, str) or not self.output_dir.strip():
            raise ValueError("recording.output_dir must be a nonempty path")


def load_config(path=DEFAULT_CONFIG):
    values = load_yaml_config(path)
    unknown = set(values) - {"cameras", "main_depth", "state_hz", "output_dir"}
    if unknown:
        raise ValueError(f"Unknown recording settings: {sorted(unknown)}")
    if "cameras" not in values:
        raise ValueError("recording.cameras is required")
    return RecordingConfig(**{**values, "cameras": tuple(values["cameras"])})
