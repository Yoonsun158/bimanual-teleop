"""Quest wire-format fixtures with no transport."""

import json

from bimanual_teleop.devices.quest.adapter import decode_message

def frame(sequence: int = 0, **changes: object) -> dict[str, object]:
    def pose(position: list[float], active: bool | None = True) -> dict[str, object]:
        return {"p": position, "q": [0.0, 0.0, 0.0, 1.0], "flags": 15, "active": active}

    result = {
        "v": 1, "type": "frame", "session": "test-session", "seq": sequence,
        "origin": 0, "query_ns": 1_000_000_000 + sequence * 10_000_000,
        "xr_time": 2_000_000_000 + sequence * 10_000_000,
        "send_ns": 1_001_000_000 + sequence * 10_000_000,
        "state": 5, "refresh_hz": 120.0,
        "head": pose([0.0, 1.0, 0.0], None),
        "left": pose([1.0, 2.0, 3.0]), "right": pose([-1.0, 4.0, 2.0]),
    }
    result.update(changes)
    return result


def sample(sequence: int = 0, received_ns: int = 3_000_000_000, **changes: object):
    return decode_message(json.dumps(frame(sequence, **changes)), received_ns)


def reference_event(origin: int = 1, change_time: int = 2_010_000_000) -> str:
    return json.dumps({
        "v": 1, "type": "event", "session": "test-session",
        "event": "reference_space_change", "device_ns": 123,
        "details": {"origin": origin, "change_time": change_time, "pose_valid": False,
                    "pose_in_previous_space": {"p": None, "q": None}},
    })
