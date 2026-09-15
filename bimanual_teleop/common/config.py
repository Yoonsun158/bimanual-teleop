"""Read YAML configuration shared by device entry points."""

from pathlib import Path


def load_yaml_config(path: str | Path) -> dict:
    import yaml

    with Path(path).open(encoding="utf-8") as stream:
        try:
            config = yaml.safe_load(stream)
        except yaml.YAMLError as error:
            raise ValueError(f"Invalid YAML configuration {path}: {error}") from error
    if not isinstance(config, dict):
        raise ValueError(f"Configuration must be a mapping: {path}")
    return config
