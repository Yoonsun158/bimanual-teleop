"""Read single keys from an interactive terminal without blocking a control loop."""

from __future__ import annotations

import os
import select
import sys
import termios
import tty

from bimanual_teleop.common.console import print_message


def confirm_motion(terminal, action):
    """Require a fresh Enter in an interactive terminal before starting motion."""
    # Discard keys entered before the warning, including a queued Enter.
    while True:
        keys = terminal.read(0.)
        if keys == "" or (keys and any(key.lower() in ("q", "\x03", "\x04") for key in keys)):
            print_message("已取消启动。", "done")
            return False
        if keys is None:
            break
    print_message("设备即将运动。请确认机械臂和灵巧手周围无人员、障碍物，"
                  "运动范围内无碰撞风险，实体急停已释放，并做好随时急停的准备。", "warning")
    print_message(f"确认安全后按回车键{action}；按 Q 或 Ctrl+C 取消。")
    while True:
        keys = terminal.read(.1)
        if keys == "" or (keys and any(key.lower() in ("q", "\x03", "\x04") for key in keys)):
            print_message("已取消启动。", "done")
            return False
        if keys and any(key in ("\n", "\r") for key in keys):
            return True


class NonblockingTerminal:
    def __init__(self, stream=None):
        self.stream = sys.stdin if stream is None else stream
        self._escape = None

    def __enter__(self):
        if not self.stream.isatty():
            raise ValueError("需要交互终端；请直接运行命令，不要重定向 stdin")
        self.fd = self.stream.fileno()
        self.original = termios.tcgetattr(self.fd)
        try:
            tty.setcbreak(self.fd)
        except BaseException:
            termios.tcsetattr(self.fd, termios.TCSADRAIN, self.original)
            raise
        return self

    def read(self, timeout: float):
        if not select.select([self.fd], [], [], max(0., timeout))[0]:
            return None
        data = os.read(self.fd, 32)
        return self._keys(data.decode("utf-8", errors="ignore")) if data else ""

    def _keys(self, text: str):
        keys = []
        for key in text:
            if key == "\x1b":
                self._escape = "prefix"
            elif self._escape == "prefix":
                self._escape = "sequence" if key in "[O" else None
            elif self._escape == "sequence":
                if "@" <= key <= "~":
                    self._escape = None
            else:
                keys.append(key)
        return "".join(keys) or None

    def __exit__(self, *exc):
        termios.tcsetattr(self.fd, termios.TCSADRAIN, self.original)
