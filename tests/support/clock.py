"""Deterministic monotonic clock for control tests."""


class Clock:
    def __init__(self, now=1_000_000_000):
        self.now = now

    def __call__(self):
        return self.now

    def advance(self, ns=5_000_000):
        self.now += ns

    def advance_s(self, seconds):
        self.advance(round(seconds * 1e9))
