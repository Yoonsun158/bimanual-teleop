"""Address and SDK-user options shared by Wuji viewer and calibration."""

from __future__ import annotations

from pathlib import Path

from bimanual_teleop.paths import PROJECT_ROOT
from bimanual_teleop.common.config import load_yaml_config

DEFAULT_CONFIG = PROJECT_ROOT / "configs/wuji_teleop.yaml"


def sdk_user_name(config, *, user_name=None):
    if "sdk_user_id" in config:
        raise ValueError("请将 Wuji 配置中的 sdk_user_id 改为 sdk_user_name，并填写用户名")
    selected = config.get("sdk_user_name", "") if user_name is None else user_name
    if not isinstance(selected, str) or (selected and not selected.strip()):
        raise ValueError("sdk_user_name must be a nonblank string (or empty for default)")
    return selected


def glove_settings(config_path, side, address=None, *, user_name=None):
    config = load_yaml_config(config_path)
    selected = address or config["devices"][side]["glove"]
    user = {"user_name": sdk_user_name(config, user_name=user_name)}
    if not isinstance(selected, str) or not selected:
        raise ValueError(f"missing {side} glove address")
    return selected, user


def add_glove_arguments(parser, *, include_sdk_user=True):
    parser.add_argument("--wuji-config", "--config", dest="config", type=Path, default=DEFAULT_CONFIG,
                        help="Wuji 配置；默认 configs/wuji_teleop.yaml")
    parser.add_argument("--side", choices=("left", "right"), required=True)
    parser.add_argument("--address", help="override the selected glove address")
    if include_sdk_user:
        parser.add_argument("--user-name", help="按已有 SDK 用户名选择用户；默认从配置文件读取")
