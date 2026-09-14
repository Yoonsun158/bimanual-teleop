"""Address and SDK-user options shared by Wuji viewer and calibration."""

from __future__ import annotations

from pathlib import Path

from bimanual_teleop.paths import PROJECT_ROOT
from bimanual_teleop.common.config import load_json_config

DEFAULT_CONFIG = PROJECT_ROOT / "configs/wuji_teleop.json"


def glove_settings(config_path, side, address=None, sdk_user_id=None, *, user_name=None):
    config = load_json_config(config_path)
    selected = address or config["devices"][side]["glove"]
    if user_name is not None or sdk_user_id is not None:
        user = {"user_name": user_name or "", "user_id": sdk_user_id or ""}
    else:
        user = {"user_name": config.get("sdk_user_name", ""),
                "user_id": config.get("sdk_user_id", "")}
    if not isinstance(selected, str) or not selected:
        raise ValueError(f"missing {side} glove address")
    for key, value in user.items():
        if not isinstance(value, str) or (value and not value.strip()):
            raise ValueError(f"sdk_{key} must be a nonblank string (or empty for default)")
    if user["user_name"] and user["user_id"]:
        raise ValueError("specify only one of sdk_user_name or sdk_user_id")
    return selected, user


def add_glove_arguments(parser, *, include_sdk_user=True):
    parser.add_argument("--wuji-config", "--config", dest="config", type=Path, default=DEFAULT_CONFIG,
                        help="Wuji 配置；默认 configs/wuji_teleop.json")
    parser.add_argument("--side", choices=("left", "right"), required=True)
    parser.add_argument("--address", help="override the selected glove address")
    if include_sdk_user:
        user = parser.add_mutually_exclusive_group()
        user.add_argument("--user-name", help="按已有 SDK 用户名选择用户；默认从配置文件读取")
        user.add_argument("--user-id", "--sdk-user-id", dest="sdk_user_id",
                          help="兼容旧配置或区分同名用户的实际 SDK ID")
