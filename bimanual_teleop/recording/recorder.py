"""One recording process, isolated from both robot control interpreters."""

from dataclasses import asdict, replace
from datetime import datetime
import multiprocessing as mp
from pathlib import Path
from queue import Empty
import time
from uuid import uuid4

from .sink import CaptureChannel, RecorderSink, STATE_STREAMS, COMMAND_STREAMS


class _Preview:
    def __init__(self):
        import matplotlib.pyplot as plt
        import numpy as np
        if plt.get_backend().lower() == "agg":
            raise RuntimeError("录制预览需要桌面图形环境")
        self.plt = plt
        self.figure, axes = plt.subplots(1, 3, figsize=(15, 4))
        self.artists = {}
        for index, axis in enumerate(axes):
            axis.set_title(f"camera_{index}")
            axis.set_axis_off()
            self.artists[f"camera_{index}"] = axis.imshow(np.zeros((480, 640, 3), dtype="u1"))
        plt.show(block=False)
        self.next_update = 0.

    def update(self, frames):
        if not self.plt.fignum_exists(self.figure.number) or time.monotonic() < self.next_update:
            return
        for camera, image in frames.items():
            self.artists[camera].set_data(image)
        self.figure.canvas.draw_idle()
        self.plt.pause(.001)
        self.next_update = time.monotonic() + .1


def _worker(config, sdk_root, metadata, channel, connection, viewer):
    from bimanual_teleop.devices.tianji.model import TianjiKinematics
    from .camera import CameraRig
    from .storage import EpisodeWriter

    rig = CameraRig(config)
    writer = None
    generation = 0
    ending = None
    latest_images = {}
    camera_seen = {}
    required_cameras = [f"cameras/camera_{i}/rgb" for i in range(3)]
    if config.main_depth:
        required_cameras.append("cameras/camera_0/depth")
    try:
        kinematics = TianjiKinematics(sdk_root)
        preview = _Preview() if viewer else None
        rig.start()
        metadata = {**metadata, "recording": asdict(config), "cameras": rig.metadata,
                    "model_sha256": kinematics.model.digest,
                    "pose_source": "FK of recorded measured joints; xyz_m + quaternion_xyzw",
                    "arm_frames": {s: [f"tianji_{s}_base", f"tianji_{s}_flange"]
                                   for s in ("left", "right")},
                    "joint_order": {"arms": "left/right: J1..J7",
                                    "hands": "left/right: finger1..5, each joint1..4"},
                    "wrench": "Fx,Fy,Fz [N], Tx,Ty,Tz [Nm]; native sensor axes, no added compensation",
                    "state_time": "host SDK observation/dequeue monotonic_ns",
                    "command_time": "successful SDK submission; not physical execution",
                    "camera_time": "GLOBAL_TIME frame timestamp mapped to host monotonic; not exposure midpoint"}
        connection.send(("ready", None))
        running = True
        while running:
            parent = mp.parent_process()
            if parent is not None and not parent.is_alive():
                raise RuntimeError("遥操作主进程已退出")
            while connection.poll():
                message = connection.recv()
                operation = message[0]
                if operation == "start":
                    if writer is not None:
                        raise RuntimeError("Previous episode has not finished")
                    _, generation, path, start_ns = message
                    writer = EpisodeWriter(path, start_ns, metadata, kinematics)
                    ending = None
                    channel.active.value = True
                    connection.send(("recording", str(path)))
                elif operation == "stop" and writer is not None:
                    channel.active.value = False
                    _, end_ns, status, reason = message
                    ending = (end_ns, status, reason, time.monotonic() + .15)
                elif operation == "close":
                    running = False
            for _ in range(128):
                try:
                    record_generation, record = channel.queue.get_nowait()
                except Empty:
                    break
                channel.consumed[(STATE_STREAMS + COMMAND_STREAMS).index(record.stream)] += 1
                if writer is not None and record_generation == generation and (
                        ending is None or record.time_ns <= ending[0]):
                    writer.append(record)
            for camera, kind, image, record in rig.poll():
                camera_seen[record.stream] = record.time_ns
                if kind == "rgb":
                    latest_images[camera] = image
                if writer is not None and (ending is None or record.time_ns <= ending[0]):
                    if kind == "rgb":
                        writer.write_rgb(camera, image, record)
                    else:
                        writer.append(replace(record, values={**record.values, "image": image}))
            if preview is not None:
                preview.update(latest_images)
            drained = (not any(channel.inflight) and list(channel.sent) == list(channel.consumed))
            past_end = ending is not None and all(camera_seen.get(s, 0) >= ending[0]
                                                   for s in required_cameras)
            if ending is not None and time.monotonic() >= ending[3] and (
                    drained and past_end or time.monotonic() >= ending[3] + 2.):
                end_ns, status, reason, _ = ending
                missing = set(STATE_STREAMS + COMMAND_STREAMS + tuple(required_cameras)) - set(writer.counts)
                if not drained or not past_end or missing:
                    status = "failed"
                    reason = f"录制数据未完整收尾；缺少流：{sorted(missing)}"
                    channel.fail(reason)
                if channel.failed.is_set():
                    status, reason = "failed", reason or "采集通道失败"
                path = str(writer.path)
                writer.close(end_ns, status, reason)
                writer = None
                ending = None
                channel.active.value = False
                connection.send(("saved", (path, status)))
            time.sleep(.001)
    except BaseException as error:
        channel.fail(f"录制进程失败：{error}")
    finally:
        channel.active.value = False
        if writer is not None:
            try:
                writer.close(time.monotonic_ns(), "failed", "录制未正常完成")
            except Exception:
                pass  # Disk failure leaves the manifest incomplete, never complete.
        rig.close()
        connection.close()


class Recorder:
    """UI coordinator. begin/end are asynchronous during teleoperation."""

    def __init__(self, config, *, sdk_root=None, metadata=None, viewer=False):
        self.config, self.sdk_root = config, sdk_root
        self.metadata, self.viewer = metadata or {}, viewer
        self.context = mp.get_context("spawn")
        self.channel = CaptureChannel(self.context)
        self.sink = RecorderSink(self.channel, config.state_hz)
        self.session = Path(config.output_dir).resolve() / (
            datetime.now().strftime("%Y%m%d_%H%M%S_") + uuid4().hex[:8])
        self.process = self.connection = None
        self.ready = False
        self.state = "idle"
        self.error = None
        self.notices = []
        self._restart_required = False

    @property
    def recording(self):
        return self.state in ("starting", "recording")

    def _launch(self):
        self.channel.failed.clear()
        self.error = None
        # Keep cumulative counts: another process's Queue feeder may still hold
        # old records. The worker consumes them but writes only its episode.
        while True:
            try:
                self.channel.errors.get_nowait()
            except Empty:
                break
        parent, child = self.context.Pipe()
        self.connection = parent
        self.process = self.context.Process(target=_worker,
            args=(self.config, self.sdk_root, self.metadata, self.channel, child, self.viewer),
            name="demonstration-recorder")
        self.process.start()
        child.close()
        self.ready = False
        self.state = "idle"

    def start(self):
        """Camera preflight before teleoperation is engaged."""
        self._launch()
        deadline = time.monotonic() + 15.
        while not self.ready:
            self.poll()
            if self.error:
                raise RuntimeError(self.error)
            if time.monotonic() >= deadline:
                raise RuntimeError("采集相机启动超时")
            time.sleep(.01)

    def begin(self):
        if self.state != "idle":
            raise RuntimeError("请等待当前录制保存完成")
        if self.error:
            raise ValueError("请暂停遥操作后恢复采集进程")
        if not self.ready:
            raise ValueError("采集相机尚未就绪")
        now = time.monotonic_ns()
        for stream, stamp in zip(STATE_STREAMS + COMMAND_STREAMS, self.channel.latest_ns):
            if stamp == 0 or now - stamp > 100_000_000:
                raise ValueError(f"等待新鲜的采集数据：{stream}")
        self.channel.generation.value += 1
        generation = self.channel.generation.value
        path = self.session / f"episode_{generation - 1:06d}"
        self.connection.send(("start", generation, str(path), now))
        self.state = "starting"

    def recover(self):
        """Called only while the UI has paused motion; joining cannot stall control."""
        self.channel.active.value = False
        self._stop_process()
        if self._restart_required:
            self.ready = False
            self.state = "idle"
            self.error = "采集进程被强制终止，通信队列可能损坏；请退出并重新启动遥操作"
            return
        self._launch()

    def end(self, *, status="complete", reason=None):
        if not self.recording:
            return
        self.channel.active.value = False
        if self.process is not None and self.process.is_alive():
            try:
                self.connection.send(("stop", time.monotonic_ns(), status, reason))
                self.state = "saving"
            except (OSError, EOFError):
                self.state = "idle"
                self.error = "录制进程停止通道已断开"
                self.channel.fail(self.error)
        else:
            self.state = "idle"

    def poll(self):
        if self.connection is not None:
            try:
                while self.connection.poll():
                    kind, value = self.connection.recv()
                    if kind == "ready":
                        self.ready = True
                        self.notices.append("三路相机采集已就绪。")
                    elif kind == "recording":
                        if self.state == "starting":
                            self.state = "recording"
                        self.notices.append(f"正在录制：{value}")
                    elif kind == "saved":
                        self.state = "idle"
                        path, status = value
                        label = {"complete": "已保存", "discarded": "已作废"}.get(status, "不完整")
                        self.notices.append(f"录制{label}：{path}")
            except (EOFError, OSError):
                pass
        while True:
            try:
                self.error = self.channel.errors.get_nowait()
            except Empty:
                break
        if self.channel.failed.is_set():
            self.error = self.error or "采集通道失败"
        if self.process is not None and not self.process.is_alive():
            self.ready = False
            self.state = "idle"
            self.error = self.error or "录制进程已退出"
        if self.recording and not self.error:
            now = time.monotonic_ns()
            for stream, stamp in zip(STATE_STREAMS + COMMAND_STREAMS, self.channel.latest_ns):
                if now - stamp > 500_000_000:
                    self.channel.fail(f"采集数据断流：{stream}")
                    self.error = f"采集数据断流：{stream}"
                    break
        return self.error

    def _stop_process(self):
        if self.process is None:
            return
        if self.process.is_alive():
            try:
                self.connection.send(("close",))
            except (OSError, EOFError):
                pass
            self.process.join(3.)
        if self.process.is_alive():
            self._restart_required = True
            self.process.terminate()
            self.process.join(2.)
        if self.process.is_alive():
            self.process.kill()
            self.process.join(1.)
        if self.process.exitcode not in (None, 0):
            self._restart_required = True
        self.connection.close()
        self.process = self.connection = None

    def close(self):
        self.end(status="failed", reason="退出时录制尚未结束")
        deadline = time.monotonic() + 3.
        while self.state == "saving" and time.monotonic() < deadline:
            self.poll()
            time.sleep(.01)
        self._stop_process()
        self.channel.queue.close()
        self.channel.queue.cancel_join_thread()
        self.channel.errors.close()
        self.channel.errors.cancel_join_thread()
