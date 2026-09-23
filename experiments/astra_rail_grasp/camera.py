"""Isolated RGB/depth acquisition; importing this module never opens hardware."""

from __future__ import annotations

import json
import math
import multiprocessing as mp
from pathlib import Path
import queue
import re
import threading
import time

import numpy as np
from PIL import Image, ImageDraw


MAX_AGE_S = 0.5


def _intrinsics(frame):
    intr = frame.profile.as_video_stream_profile().get_intrinsics()
    return {"width": intr.width, "height": intr.height, "fx": intr.fx,
            "fy": intr.fy, "ppx": intr.ppx, "ppy": intr.ppy,
            "model": str(intr.model), "coeffs": list(intr.coeffs)}


def _depth_meters(raw, scale):
    """Missing/zero/nonfinite ranges are unknown, never a zero-distance point."""
    depth = np.asarray(raw, dtype=np.float32) * float(scale)
    valid = np.isfinite(depth) & (depth > 0)
    if not math.isfinite(scale) or scale <= 0:
        valid[:] = False
    depth[~valid] = np.nan
    return depth, float(np.count_nonzero(valid) / depth.size) if depth.size else 0.0


def _open_camera(rs, context, serial):
    errors = []
    for depth_enabled in (True, False):
        pipeline = rs.pipeline(context)
        try:
            config = rs.config()
            config.enable_device(serial)
            config.enable_stream(rs.stream.color, 640, 480, rs.format.rgb8, 30)
            if depth_enabled:
                config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
            profile = pipeline.start(config)
            scale = (float(profile.get_device().first_depth_sensor().get_depth_scale())
                     if depth_enabled else None)
            return {"pipeline": pipeline, "align": rs.align(rs.stream.color) if depth_enabled else None,
                    "depth_scale": scale, "depth_enabled": depth_enabled,
                    "depth_problem": "; ".join(errors) or None}
        except Exception as error:
            errors.append(f"{'RGB+depth' if depth_enabled else 'RGB'}: {error}")
            try:
                pipeline.stop()
            except Exception:
                pass
    return {"problem": "; ".join(errors)}


def _extract_frame(camera, frames, sequence):
    # This is a host dequeue observation, not the sensor exposure timestamp.
    observed_ns = time.monotonic_ns()
    problem = camera.get("depth_problem")
    if camera["align"] is not None:
        try:
            frames = camera["align"].process(frames)
        except Exception as error:
            return None, f"depth alignment failed: {error}"
    color = frames.get_color_frame()
    if not color:
        return None, None
    result = {"sequence": sequence, "captured_monotonic_ns": observed_ns,
              "host_timestamp_meaning": "host monotonic observation of dequeued frames",
              "frame_number": int(color.get_frame_number()),
              "device_timestamp_ms": float(color.get_timestamp()),
              "device_timestamp_domain": str(color.get_frame_timestamp_domain()),
              "intrinsics": _intrinsics(color), "depth_scale": camera["depth_scale"],
              "depth_valid_fraction": None, "depth_frame_number": None,
              "depth_device_timestamp_ms": None, "depth_intrinsics": None,
              "rgb": np.asanyarray(color.get_data()).copy(), "depth": None}
    depth = frames.get_depth_frame() if camera["depth_enabled"] else None
    if depth:
        result["depth"], result["depth_valid_fraction"] = _depth_meters(
            np.asanyarray(depth.get_data()), camera["depth_scale"])
        result.update(depth_frame_number=int(depth.get_frame_number()),
                      depth_device_timestamp_ms=float(depth.get_timestamp()),
                      depth_intrinsics=_intrinsics(depth))
        if result["depth_valid_fraction"] == 0:
            problem = "aligned depth contains no valid range; distance is unknown"
    elif camera["depth_enabled"]:
        problem = "current color frame has no aligned depth; distance is unknown"
    else:
        problem = problem or "depth unavailable; RGB only"
    return result, problem


def _publish_latest(out_queue, packet):
    """Bound memory and latency; old packets may be superseded as complete units."""
    try:
        out_queue.put_nowait(packet)
    except queue.Full:
        try:
            out_queue.get_nowait()
        except queue.Empty:
            return
        try:
            out_queue.put_nowait(packet)
        except queue.Full:
            pass


def _simulated_frame(sequence):
    image = Image.new("RGB", (640, 480), (28, 30, 48))
    draw = ImageDraw.Draw(image)
    draw.rectangle((20, 20, 620, 460), outline=(255, 180, 0), width=5)
    draw.text((70, 170), "SIMULATED - NO REAL CAMERA", fill=(255, 230, 0), font_size=28)
    draw.text((85, 225), "TEST IMAGE ONLY - NOT THE ROBOT WORKSPACE", fill="white", font_size=18)
    depth = np.full((480, 640), 1.0, dtype=np.float32)
    depth[:40] = np.nan
    return {"sequence": sequence, "captured_monotonic_ns": time.monotonic_ns(),
            "host_timestamp_meaning": "simulated frame generation",
            "frame_number": sequence, "device_timestamp_ms": None,
            "device_timestamp_domain": "SIMULATED", "intrinsics": None,
            "depth_scale": 0.001, "depth_valid_fraction": float(np.isfinite(depth).mean()),
            "depth_frame_number": sequence, "depth_device_timestamp_ms": None,
            "depth_intrinsics": None, "rgb": np.asarray(image), "depth": depth}


def _capture_worker(out_queue, stopped, fake):
    cameras, latest = {}, {}
    out_queue.cancel_join_thread()
    try:
        if fake:
            sequence = 0
            while not stopped.is_set():
                sequence += 1
                _publish_latest(out_queue, {"cameras": {"SIMULATED-0": {
                    "serial": "SIMULATED-0", "name": "SIMULATED TEST CAMERA",
                    "simulated": True, "problem": None, "depth_enabled": True,
                    "frame": _simulated_frame(sequence)}}, "problems": []})
                stopped.wait(1 / 30)
            return
        import pyrealsense2 as rs

        context = rs.context()
        for device in context.query_devices():
            serial = device.get_info(rs.camera_info.serial_number)
            name = device.get_info(rs.camera_info.name)
            camera = _open_camera(rs, context, serial)
            cameras[serial] = camera
            latest[serial] = {"serial": serial, "name": name, "simulated": False,
                              "depth_enabled": camera.get("depth_enabled", False),
                              "problem": camera.get("problem"), "frame": None}
        if not cameras:
            while not stopped.is_set():
                _publish_latest(out_queue, {"cameras": {}, "problems": ["No RealSense cameras found"]})
                stopped.wait(.1)
            return
        last_publish = 0.0
        while not stopped.is_set():
            changed = False
            for serial, camera in cameras.items():
                if "pipeline" not in camera or camera.get("failed"):
                    continue
                try:
                    frames = camera["pipeline"].poll_for_frames()
                    if not frames:
                        continue
                    previous = latest[serial]["frame"]
                    frame, problem = _extract_frame(camera, frames,
                                                     1 if previous is None else previous["sequence"] + 1)
                    if (frame is not None and previous is not None
                            and frame["frame_number"] == previous["frame_number"]
                            and frame["device_timestamp_ms"] == previous["device_timestamp_ms"]):
                        frame = None
                    # Replace whole records: Queue's feeder serializes asynchronously.
                    latest[serial] = {**latest[serial], "problem": problem,
                                      "frame": frame if frame is not None else previous}
                    changed = True
                except Exception as error:
                    camera["failed"] = True
                    latest[serial] = {**latest[serial], "stream_failed": True,
                                      "problem": f"camera stream failed: {error}"}
                    changed = True
            now = time.monotonic()
            if changed or now - last_publish >= 0.1:
                _publish_latest(out_queue, {"cameras": dict(latest), "problems": []})
                last_publish = now
            stopped.wait(0.005)
    except Exception as error:
        # Keep terminal errors observable until the parent closes the worker.
        for serial in latest:
            latest[serial] = {**latest[serial], "stream_failed": True}
        while not stopped.is_set():
            _publish_latest(out_queue, {"cameras": dict(latest),
                                        "problems": [f"camera acquisition unavailable: {error}"]})
            stopped.wait(.1)
    finally:
        for camera in cameras.values():
            if "pipeline" in camera:
                try:
                    camera["pipeline"].stop()
                except Exception:
                    pass


class CameraCapture:
    """Parent-side latest-frame cache. Only snapshot() writes artifacts."""

    def __init__(self, output_dir: Path, fake=False):
        self.output_dir = Path(output_dir).resolve()
        self.fake = bool(fake)
        self._process = self._queue = self._stopped = None
        self._cameras = {}
        self._problems = ["camera acquisition has not started"]
        self._snapshot_sequence = 0
        self._closed = False
        self._cache_lock = threading.Lock()
        self._reader_stop = threading.Event()
        self._reader = None

    def start(self):
        if self._closed:
            self._problems = ["camera acquisition is closed"]
            return
        if self._process is not None:
            return
        try:
            context = mp.get_context("spawn")
            self._queue = context.Queue(maxsize=2)
            self._stopped = context.Event()
            self._process = context.Process(target=_capture_worker,
                                            args=(self._queue, self._stopped, self.fake),
                                            name="astra-rail-camera", daemon=True)
            self._process.start()
            self._problems = ["waiting for first camera frame"]
            self._reader = threading.Thread(target=self._receive, name="astra-camera-cache", daemon=True)
            self._reader.start()
        except Exception as error:
            self._process = None
            self._problems = [f"camera process could not start: {error}"]

    def _accept_packet(self, packet):
        with self._cache_lock:
            self._cameras = packet["cameras"]
            self._problems = packet.get("problems", [])

    def _receive(self):
        # Deserializing large image packets never happens in health/control calls.
        while not self._reader_stop.is_set():
            try:
                self._accept_packet(self._queue.get(timeout=.1))
            except queue.Empty:
                continue
            except (EOFError, OSError, ValueError) as error:
                with self._cache_lock:
                    self._problems = [f"camera IPC unavailable: {error}"]
                break

    def _cached(self):
        with self._cache_lock:
            return dict(self._cameras), list(self._problems)

    def _health(self, cached, initial_problems):
        now = time.monotonic_ns()
        cameras, problems = [], list(initial_problems)
        worker_dead = self._process is not None and not self._process.is_alive()
        if worker_dead and not self._closed:
            problems.append(f"camera process exited (code {self._process.exitcode})")
        if self._closed:
            problems.append("camera acquisition is closed")
        for serial, camera in sorted(cached.items()):
            frame = camera.get("frame")
            item = {key: value for key, value in camera.items() if key != "frame"}
            if frame is not None:
                item.update({key: value for key, value in frame.items() if key not in ("rgb", "depth")})
                item["age_s"] = max(0., (now - frame["captured_monotonic_ns"]) / 1e9)
                item["ready"] = (item["age_s"] <= MAX_AGE_S and not self._closed
                                 and not worker_dead and not item.get("stream_failed", False))
                if item["age_s"] > MAX_AGE_S:
                    problems.append(f"{serial}: latest RGB frame is stale ({item['age_s']:.3f} s)")
            else:
                item.update(age_s=None, ready=False)
                problems.append(f"{serial}: no RGB frame available")
            if item.get("problem"):
                problems.append(f"{serial}: {item['problem']}")
            cameras.append(item)
        return {"ready": any(item["ready"] for item in cameras), "simulated": self.fake,
                "observed_monotonic_ns": now, "cameras": cameras,
                "problems": list(dict.fromkeys(problems))}

    def health(self):
        return self._health(*self._cached())

    def snapshot(self):
        cached, problems = self._cached()
        result = self._health(cached, problems)
        self._snapshot_sequence += 1
        result["snapshot_sequence"] = self._snapshot_sequence
        result["json_path"] = None
        try:
            self.output_dir.mkdir(parents=True, exist_ok=True)
            stamp = f"snapshot_{time.time_ns()}_{self._snapshot_sequence:06d}"
            for item in result["cameras"]:
                item.update(rgb_path=None, depth_path=None)
                frame = cached[item["serial"]].get("frame")
                if frame is None:
                    continue
                serial = re.sub(r"[^a-zA-Z0-9_-]", "_", item["serial"])
                prefix = self.output_dir / f"{stamp}_{serial}"
                image_path = Path(f"{prefix}_rgb.png")
                Image.fromarray(frame["rgb"]).save(image_path)
                item["rgb_path"] = str(image_path)
                if frame["depth"] is not None:
                    depth_path = Path(f"{prefix}_depth_m.npy")
                    np.save(depth_path, frame["depth"], allow_pickle=False)
                    item["depth_path"] = str(depth_path)
            json_path = self.output_dir / f"{stamp}.json"
            result["json_path"] = str(json_path)
            json_path.write_text(json.dumps(result, ensure_ascii=False, indent=2, allow_nan=False),
                                 encoding="utf-8")
        except Exception as error:
            result["problems"].append(f"camera snapshot could not be saved: {error}")
            result["json_path"] = None
        return result

    def close(self):
        if self._closed:
            return
        self._closed = True
        self._reader_stop.set()
        if self._stopped is not None:
            self._stopped.set()
        if self._process is not None:
            self._process.join(timeout=1.)
            if self._process.is_alive():
                self._process.terminate()
                self._process.join(timeout=0.5)
        if self._reader is not None:
            self._reader.join(timeout=.3)
        if self._queue is not None:
            self._queue.cancel_join_thread()
            self._queue.close()
