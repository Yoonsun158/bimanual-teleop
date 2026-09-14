"""Read single keys from an interactive terminal without blocking a control loop."""

from __future__ import annotations

import os
import select
import sys
import termios
import tty


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
