"""Shared Tianji device, motion, ready-pose and Quest settings."""

from __future__ import annotations

import ipaddress
from pathlib import Path

from bimanual_teleop.paths import PROJECT_ROOT
from bimanual_teleop.common.config import load_json_config


DEFAULT_CONFIG = PROJECT_ROOT / "configs/tianji_teleop.json"


def load_config(path: str | Path = DEFAULT_CONFIG, override: str | None = None) -> dict:
    source = load_json_config(path)
    if not isinstance(source, dict):
        raise ValueError("Tianji configuration must be an object")
    address = source.get("controller_ip") if override is None else override
    if not isinstance(address, str):
        raise ValueError("Tianji controller_ip must be an IPv4 address")
    try:
        source["controller_ip"] = str(ipaddress.IPv4Address(address))
    except ipaddress.AddressValueError as error:
        raise ValueError(f"invalid Tianji controller IPv4 address: {address!r}") from error
    quest = source.setdefault("quest", {})
    if quest.setdefault("coordinate_frame", "headset") not in ("headset", "world"):
        raise ValueError("quest.coordinate_frame must be headset or world")
    if "profile" in source:
        profile = source["profile"]
        if not isinstance(profile, dict):
            raise ValueError("Tianji profile must be an object")
        profile.setdefault("profile_id", "tianji-teleop")
        profile.setdefault("mode", "cartesian_impedance")
    return source
