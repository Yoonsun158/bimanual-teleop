"""Convert complete raw episodes to the official DP ReplayBuffer layout.

All vectors concatenate left before right. Poses are xyz (m) followed by
rotation vectors (rad); joints are radians and wrench is Fx,Fy,Fz (N),
Tx,Ty,Tz (N m). Actions concatenate both arms, then both hands. Cartesian
actions use the recorded controller input goals, never FK of joint commands.
"""

from __future__ import annotations

from dataclasses import dataclass
import json
import os
from pathlib import Path
import shutil
import tempfile


STATE_GAP_NS = 50_000_000
COMMAND_AGE_NS = 50_000_000
CAMERA_TOLERANCE_NS = 20_000_000
SIDES = ("left", "right")


@dataclass
class _Stream:
    times: object
    values: dict
    rows: object
    raw_count: int


def _read_stream(raw, name, fields, start_ns, end_ns):
    import numpy as np

    if name not in raw:
        raise ValueError(f"Missing required raw stream: {name}")
    group = raw[name]
    times = np.asarray(group["time_ns"][:])
    sequence = np.asarray(group["sequence"][:])
    if (times.ndim != 1 or times.dtype != np.dtype("int64")
            or sequence.shape != times.shape or sequence.dtype != np.dtype("int64")):
        raise ValueError(f"{name}: time_ns and sequence must be equally sized int64 vectors")
    if len(times) > 1 and np.any(times[1:] <= times[:-1]):
        raise ValueError(f"{name}: time_ns must be strictly increasing")
    rows = np.flatnonzero((times >= start_ns) & (times < end_ns))
    values = {}
    for field, width in fields.items():
        array = group[field]
        if array.shape != (len(times), width):
            raise ValueError(f"{name}/{field}: expected shape ({len(times)}, {width})")
        values[field] = np.asarray(array[:], dtype=np.float64)[rows]
    if name.startswith("cameras/"):
        if group["source_time_ms"].shape != times.shape:
            raise ValueError(f"{name}: source_time_ms length differs from timestamps")
    return _Stream(times[rows], values, rows, len(times))


def _brackets(times, query):
    import numpy as np

    if len(times) == 0:
        return np.zeros(len(query), int), np.zeros(len(query), int), np.zeros(len(query), bool)
    right = np.searchsorted(times, query, side="left")
    hi = np.clip(right, 0, len(times) - 1)
    exact = times[hi] == query
    lo = np.where(exact, hi, np.clip(right - 1, 0, len(times) - 1))
    valid = (query >= times[0]) & (query <= times[-1])
    valid &= times[hi] - times[lo] <= STATE_GAP_NS
    return lo, hi, valid


def _interpolate(stream, query, field, *, pose=False):
    import numpy as np

    values = stream.values[field]
    width = 7 if pose else values.shape[1]
    if not len(stream.times):
        return np.zeros((len(query), width)), np.zeros(len(query), bool)
    lo, hi, valid = _brackets(stream.times, query)
    a, b = values[lo], values[hi]
    valid &= np.isfinite(a).all(axis=1) & np.isfinite(b).all(axis=1)
    gap = stream.times[hi] - stream.times[lo]
    weight = np.divide(query - stream.times[lo], gap,
                       out=np.zeros(len(query), float), where=gap != 0)
    weight = np.clip(weight, 0., 1.)
    result = a + weight[:, None] * (b - a)
    if pose:
        # Pairwise shortest-path SLERP also handles q and -q identically.
        qa, qb = a[:, 3:].copy(), b[:, 3:].copy()
        na, nb = np.linalg.norm(qa, axis=1), np.linalg.norm(qb, axis=1)
        good = valid & np.isfinite(na) & np.isfinite(nb) & (na > 1e-8) & (nb > 1e-8)
        qa[~good] = qb[~good] = (0., 0., 0., 1.)
        qa /= np.linalg.norm(qa, axis=1)[:, None]
        qb /= np.linalg.norm(qb, axis=1)[:, None]
        dot = np.sum(qa * qb, axis=1)
        qb[dot < 0] *= -1
        theta = np.arccos(np.clip(np.abs(dot), 0., 1.))
        denom = np.sin(theta)
        wa, wb = 1. - weight, weight.copy()
        curved = denom > 1e-6
        wa[curved] = np.sin((1. - weight[curved]) * theta[curved]) / denom[curved]
        wb[curved] = np.sin(weight[curved] * theta[curved]) / denom[curved]
        quat = wa[:, None] * qa + wb[:, None] * qb
        quat /= np.linalg.norm(quat, axis=1)[:, None]
        result[:, 3:] = quat
        valid = good
    return result, valid


def _hold(stream, query, field):
    import numpy as np

    values = stream.values[field]
    if not len(stream.times):
        return np.zeros((len(query), values.shape[1])), np.zeros(len(query), bool)
    indices = np.searchsorted(stream.times, query, side="right") - 1
    safe = np.maximum(indices, 0)
    result = values[safe]
    valid = (indices >= 0) & (query - stream.times[safe] <= COMMAND_AGE_NS)
    valid &= np.isfinite(result).all(axis=1)
    return result, valid


def _pose_vector(poses):
    import numpy as np
    from scipy.spatial.transform import Rotation

    norms = np.linalg.norm(poses[:, 3:], axis=1)
    valid = np.isfinite(poses).all(axis=1) & np.isfinite(norms) & (norms > 1e-8)
    result = np.zeros((len(poses), 6), dtype=np.float64)
    result[valid, :3] = poses[valid, :3]
    if np.any(valid):
        result[valid, 3:] = Rotation.from_quat(poses[valid, 3:]).as_rotvec()
    return result, valid


def _nearest(stream, query):
    import numpy as np

    if not len(stream.times):
        return np.zeros(len(query), int), np.zeros(len(query), bool)
    right = np.clip(np.searchsorted(stream.times, query), 0, len(stream.times) - 1)
    left = np.maximum(right - 1, 0)
    indices = np.where(abs(stream.times[left] - query) <= abs(stream.times[right] - query), left, right)
    return stream.rows[indices], abs(stream.times[indices] - query) <= CAMERA_TOLERANCE_NS


def _segments(valid, times):
    import numpy as np

    keep = np.flatnonzero(valid)
    if not len(keep):
        return []
    breaks = (np.diff(keep) != 1) | (np.diff(times[keep]) > STATE_GAP_NS)
    return np.split(keep, np.flatnonzero(breaks) + 1)


def _append(data, key, values):
    from numcodecs import Blosc

    if key not in data:
        data.create_dataset(key, shape=(0,) + values.shape[1:], dtype=values.dtype,
                            chunks=(1024,) + values.shape[1:],
                            compressor=Blosc(cname="zstd", clevel=3, shuffle=Blosc.BITSHUFFLE))
    if data[key].shape[1:] != values.shape[1:]:
        raise ValueError(f"Output shape changed for {key}")
    data[key].append(values, axis=0)


def _image_array(data, key, total, shape, dtype):
    from numcodecs import Blosc

    if key not in data:
        return data.create_dataset(key, shape=(total,) + tuple(shape), dtype=dtype,
                                   chunks=(1,) + tuple(shape),
                                   compressor=Blosc(cname="lz4", clevel=3, shuffle=Blosc.NOSHUFFLE))
    array = data[key]
    if array.shape[1:] != tuple(shape) or array.dtype != dtype:
        raise ValueError(f"Output image format changed for {key}")
    array.resize((total,) + tuple(shape))
    return array


def _copy_video(path, expected_count, source_rows, data, key, offset):
    import av
    import numpy as np

    destinations = {}
    for i, source_row in enumerate(source_rows):
        destinations.setdefault(int(source_row), []).append(offset + i)
    count = 0
    array = None
    with av.open(str(path)) as container:
        for index, frame in enumerate(container.decode(video=0)):
            count += 1
            if index not in destinations:
                continue
            rgb = frame.to_ndarray(format="rgb24")
            if array is None:
                array = _image_array(data, key, offset + len(source_rows), rgb.shape, np.dtype("uint8"))
            if rgb.shape != array.shape[1:]:
                raise ValueError(f"Video resolution changes within {path}")
            for target in destinations[index]:
                array[target] = rgb
    if count != expected_count:
        raise ValueError(f"{path}: decoded {count} frames, timestamp table has {expected_count}")


def _convert_episode(episode, descriptor, raw, output, action_space, report, source_name):
    import numpy as np

    start, end = descriptor["start_ns"], descriptor["end_ns"]
    if (not isinstance(start, int) or not isinstance(end, int) or end <= start):
        raise ValueError(f"{episode}: invalid episode start_ns/end_ns")
    cameras = [_read_stream(raw, f"cameras/camera_{i}/rgb", {}, start, end) for i in range(3)]
    main = cameras[0]
    query = main.times
    valid = np.ones(len(query), bool)
    reasons = {}

    def require(name, mask):
        nonlocal valid
        reasons[name] = int(np.count_nonzero(~mask))
        valid &= mask

    image_rows = [main.rows]
    for i in (1, 2):
        rows, mask = _nearest(cameras[i], query)
        image_rows.append(rows)
        require(f"camera_{i}_unmatched", mask)
    values = {key: [] for key in ("robot_joint", "robot_eef_pose", "hand_joint", "wrench")}
    arm_actions, hand_actions = [], []
    for side in SIDES:
        arm = _read_stream(raw, f"arms/{side}", {"joint_pos": 7, "eef_pose": 7, "wrench": 6}, start, end)
        hand = _read_stream(raw, f"hands/{side}", {"joint_pos": 20}, start, end)
        for key, stream, field in (("robot_joint", arm, "joint_pos"), ("robot_eef_pose", arm, "eef_pose"),
                                   ("wrench", arm, "wrench"), ("hand_joint", hand, "joint_pos")):
            result, mask = _interpolate(stream, query, field, pose=field == "eef_pose")
            if field == "eef_pose":
                result, pose_valid = _pose_vector(result)
                mask &= pose_valid
            values[key].append(result)
            require(f"{side}_{key}_invalid_or_gap", mask)
        command_field, width = ("eef_pose", 7) if action_space == "eef" else ("joint_pos", 7)
        arm_command = _read_stream(raw, f"arm_commands/{side}", {command_field: width}, start, end)
        action, mask = _hold(arm_command, query, command_field)
        if action_space == "eef":
            action, pose_valid = _pose_vector(action)
            mask &= pose_valid
        arm_actions.append(action)
        require(f"{side}_arm_command_invalid_or_stale", mask)
        hand_command = _read_stream(raw, f"hand_commands/{side}", {"joint_pos": 20}, start, end)
        action, mask = _hold(hand_command, query, "joint_pos")
        hand_actions.append(action)
        require(f"{side}_hand_command_invalid_or_stale", mask)

    depth = depth_rows = None
    if "cameras/camera_0/depth" in raw:
        depth = _read_stream(raw, "cameras/camera_0/depth", {}, start, end)
        depth_rows, mask = _nearest(depth, query)
        image = raw["cameras/camera_0/depth/image"]
        if len(image.shape) != 3 or image.shape[0] != depth.raw_count or image.dtype != np.dtype("uint16"):
            raise ValueError(f"{episode}: depth must be uint16 (N,H,W), matching its timestamp table")
        require("camera_0_depth_unmatched", mask)

    runs = _segments(valid, query)
    report.update(reference_frames=len(query), valid_frames=int(valid.sum()), segments=len(runs),
                  main_camera_gaps=int(np.count_nonzero(np.diff(query) > STATE_GAP_NS)),
                  invalid_reasons={key: count for key, count in reasons.items() if count})
    if not runs:
        return []
    keep = np.flatnonzero(valid)
    data = output["data"]
    offset = data["timestamp"].shape[0] if "timestamp" in data else 0
    for key, parts in values.items():
        _append(data, key, np.concatenate(parts, axis=1)[keep].astype(np.float32))
    _append(data, "action", np.concatenate(arm_actions + hand_actions, axis=1)[keep].astype(np.float32))
    _append(data, "timestamp", (query[keep] - start).astype(np.float64) / 1e9)
    for i, camera in enumerate(cameras):
        _copy_video(episode / f"camera_{i}.mp4", camera.raw_count, image_rows[i][keep],
                    data, f"camera_{i}", offset)
    if depth is not None:
        source = raw["cameras/camera_0/depth/image"]
        target = _image_array(data, "camera_0_depth", offset + len(keep), source.shape[1:], source.dtype)
        for i, row in enumerate(depth_rows[keep]):
            target[offset + i] = source[int(row)]
    result = []
    for run in runs:
        length = len(run)
        result.append({"source_episode": source_name, "reference_frame_start": int(main.rows[run[0]]),
                       "reference_frame_end": int(main.rows[run[-1]]) + 1,
                       "start_ns": int(query[run[0]]), "end_ns": int(query[run[-1]]),
                       "output_start": offset, "output_end": offset + length})
        offset += length
    return result


def convert_recordings(input_path, output_path, *, action_space):
    """Create a new dataset; never overwrite an existing file or directory.

    Episode bounds are [start_ns,end_ns). State interpolation requires real
    brackets at most 50 ms apart. Commands are held for at most 50 ms.
    No interpolation, held command or image match crosses an episode boundary.
    Returns the quality report, also stored in meta.attrs['quality_report'].
    """
    import numpy as np
    import zarr

    if action_space not in ("eef", "joint"):
        raise ValueError("action_space must be 'eef' or 'joint'")
    source, destination = Path(input_path).expanduser().resolve(), Path(output_path).expanduser().absolute()
    if not source.is_dir():
        raise ValueError(f"Input is not a directory: {source}")
    if destination.exists() or destination.is_symlink():
        raise FileExistsError(f"Refusing to overwrite {destination}")
    manifests = [source / "episode.json"] if (source / "episode.json").is_file() else sorted(source.rglob("episode.json"))
    if not manifests:
        raise ValueError(f"No episode.json found under {source}")
    destination.parent.mkdir(parents=True, exist_ok=True)
    temporary = Path(tempfile.mkdtemp(prefix=f".{destination.name}.converting-", dir=destination.parent))
    report = {"episodes": [], "action_space": action_space}
    try:
        output = zarr.open_group(str(temporary), mode="w")
        output.create_group("data")
        meta = output.create_group("meta")
        segments = []
        has_depth = None
        for manifest in manifests:
            descriptor = json.loads(manifest.read_text(encoding="utf-8"))
            name = str(manifest.parent.relative_to(source))
            item = {"source_episode": name, "status": descriptor.get("status", "unknown")}
            report["episodes"].append(item)
            if item["status"] != "complete":
                continue
            if descriptor.get("schema_version") != 1:
                raise ValueError(f"Unsupported raw schema in {manifest}")
            raw = zarr.open_group(str(manifest.parent / "raw.zarr"), mode="r")
            this_depth = "cameras/camera_0/depth" in raw
            if has_depth is not None and this_depth != has_depth:
                raise ValueError("Complete episodes must consistently include or omit camera_0 depth")
            has_depth = this_depth
            item["metadata"] = descriptor.get("metadata", raw.attrs.get("metadata", {}))
            segments.extend(_convert_episode(manifest.parent, descriptor, raw, output, action_space, item, name))
        if not segments:
            raise ValueError("No valid frames in complete episodes")
        ends = np.asarray([segment["output_end"] for segment in segments], dtype=np.int64)
        meta.create_dataset("episode_ends", data=ends, compressor=None)
        report.update(output_episodes=len(segments), output_frames=int(ends[-1]))
        meta.attrs.update(segments=segments, quality_report=report)
        output.attrs.update(schema_version=1, format="diffusion_policy_replay_buffer", action_space=action_space,
                            side_order=list(SIDES), eef_pose_format="xyz_m+rotvec_rad", joint_unit="rad",
                            wrench_format="Fx,Fy,Fz [N]; Tx,Ty,Tz [N*m]",
                            action_layout="left_arm,right_arm,left_hand,right_hand",
                            timestamp="seconds since source episode start; camera_0 real frame times")
        # Exclusively reserve the name, then atomically replace our empty
        # reservation with the complete directory on the same filesystem.
        destination.mkdir()
        try:
            os.replace(temporary, destination)
        except BaseException:
            try:
                destination.rmdir()  # Never remove contents written by someone else.
            except OSError:
                pass
            raise
        return report
    finally:
        shutil.rmtree(temporary, ignore_errors=True)
