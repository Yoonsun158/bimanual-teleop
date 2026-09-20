"""Recording keys added to the existing terminal UI, not a second control loop."""

from bimanual_teleop.cli.runtime import TeleopUI

HELP = "C 开始录制 · S 保存 · X 作废当前条（录制要求双臂双手已接合）"


class RecordingUI(TeleopUI):
    def __init__(self, *args, recorder, **kwargs):
        super().__init__(*args, **kwargs)
        self.recorder = recorder
        self._reported_recording_error = None

    def handle(self, key):
        key = key.lower()
        if key == "c":
            if self.recorder.error:
                self.abort(self.recorder.error)
                self.recorder.recover()
                self.say("正在恢复采集；相机就绪后重新接合，再按 C。")
                return
            if getattr(self.runtime.state, "value", self.runtime.state) != "engaged":
                self.say("请先接合遥操作，再按 C 开始录制。", "warning")
                return
            try:
                self.recorder.begin()
            except (OSError, RuntimeError, ValueError) as error:
                self.say(str(error), "warning")
            return
        if key in ("s", "x"):
            self.recorder.end(status="complete" if key == "s" else "discarded")
            return
        if key in (" ", "q", "\x04"):
            self.recorder.end()
        super().handle(key)

    def handle_gesture(self, command):
        if command == "pause":
            self.recorder.end()
        return super().handle_gesture(command)

    def abort(self, reason):
        try:
            self.recorder.end(status="failed", reason=reason)
        finally:
            super().abort(reason)

    def poll_operation(self):
        error = self.recorder.poll()
        if error and error != self._reported_recording_error:
            self._reported_recording_error = error
            self.abort(error)
        elif not error:
            self._reported_recording_error = None
        while self.recorder.notices:
            self.say(self.recorder.notices.pop(0))
        super().poll_operation()

    def report_runtime_pause(self):
        if getattr(self.runtime.state, "value", self.runtime.state) == "paused":
            self.recorder.end(status="failed", reason=self.runtime.last_error or "设备暂停")
        super().report_runtime_pause()

    def close(self):
        self.recorder.end(status="failed", reason="遥操作退出")
        try:
            super().close()
        finally:
            self.recorder.close()
