"""Official Wuji Glove guided hand-model and tactile calibration."""

from __future__ import annotations

import time

from bimanual_teleop.devices.wuji.adapter import (
    WujiGloveSource, WujiSdkSession, WujiTactileFrame, _sdk_module, resolve_user_id,
)


def require_tactile_744(source, *, timeout_s=5.0, clock=time.monotonic):
    """Refuse unknown or 24×32 layouts before the SDK's 24×31 trainer starts."""
    deadline = clock() + timeout_s
    while clock() < deadline:
        sample = source.get_latest_stream("tactile")
        if sample is not None:
            tactile = sample.payload
            if not sample.header.valid or not isinstance(tactile, WujiTactileFrame):
                raise RuntimeError("Wuji tactile frame is invalid; calibration stopped")
            if (tactile.rows, tactile.columns) != (24, 31):
                raise RuntimeError(f"Wuji tactile calibration requires 24×31 (744) values; "
                                   f"received {tactile.rows}×{tactile.columns}")
            return sample
        time.sleep(.01)
    raise TimeoutError("no Wuji tactile frame arrived before calibration")


def calibrate_glove(side, address, *, kind, user_name,
                    on_feedback=None, on_pose_prompt=None, sdk=None, manager=None,
                    tactile_wait_s=5.0):
    """Run the SDK's guided flow with an isolated glove and restore prior user."""
    if kind not in ("joints", "tactile"):
        raise ValueError("calibration kind must be joints or tactile")
    sdk = sdk or _sdk_module()
    manager = manager or sdk.SdkManager.instance()
    resolve_user_id(manager, user_name=user_name, create=True)
    session = WujiSdkSession(user_name=user_name, manager=manager, sdk=sdk)
    source = WujiGloveSource(side, address, manager=manager, sdk=sdk,
                             streams=("tactile",) if kind == "tactile" else ())
    try:
        session.open()
        source.start()
        if kind == "tactile":
            require_tactile_744(source, timeout_s=tactile_wait_s)
            result = source._device.calibrate_tactile_blocking(
                on_feedback=on_feedback, on_pose_prompt=on_pose_prompt)
        else:
            result = source._device.calibrate_blocking(on_feedback=on_feedback)
        if kind == "tactile" and result.get("installed") is not True:
            raise RuntimeError("触觉数据采集结束，但 SDK 未安装触觉模型；本次标定未完成。")
        if not session.health().ready:
            raise RuntimeError("Wuji SDK user changed during calibration")
        model = result.get("calibrated_urdf" if kind == "joints" else "model_dir") or "unknown"
        return {"kind": kind, "side": side, "address": address,
                "sdk_user": dict(manager.current_user()), "model": model,
                "result": result}
    finally:
        try:
            source.close()
        finally:
            session.close()
