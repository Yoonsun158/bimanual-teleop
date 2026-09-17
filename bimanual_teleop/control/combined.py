"""Coordinate existing arm and hand runtimes without mixing their commands."""

from __future__ import annotations

from dataclasses import asdict
import threading
import time

from bimanual_teleop.system import SystemState
from bimanual_teleop.types import Health


class QuestTianjiWujiTeleop:
    """The caller ticks the arms at 200 Hz; hands own their 120 Hz worker."""

    def __init__(self, arms, hands):
        self.arms, self.hands = arms, hands
        self._state = SystemState.DISCONNECTED
        self.last_error = None
        self._generation = 0
        self._lock = threading.Lock()
        self._home_cancel = None

    @property
    def state(self):
        return self._state

    def start(self):
        try:
            self.hands.start()
            self.arms.start()
            self._state = SystemState.READY
        except BaseException:
            self.close()
            raise

    def _check_engagement(self, generation):
        if self._generation != generation or self.state == SystemState.CLOSED:
            raise RuntimeError("联合接合已取消")

    def engage(self, profile=None):
        with self._lock:
            if self.state not in (SystemState.READY, SystemState.PAUSED) or self._home_cancel is not None:
                raise RuntimeError("Pause before engaging again")
            generation = self._generation
        try:
            # Slow hand enabling happens before the arm's first-target deadline.
            self.hands.prepare_engage()
            self._check_engagement(generation)
            self.arms.engage(profile)
            self._check_engagement(generation)
            self.hands.begin_follow()
            with self._lock:
                self._check_engagement(generation)
                self.last_error = None
                self._state = SystemState.ENGAGED
        except BaseException as error:
            self.pause(str(error))
            raise

    def home(self, cancel):
        with self._lock:
            if self.state != SystemState.PAUSED or self._home_cancel is not None:
                raise RuntimeError("请先暂停遥操作再回位")
            self._home_cancel = cancel
            self._state = SystemState.HOMING
        try:
            self.hands.pause("机械臂回位期间保持手部暂停")
            self.arms.home(cancel)
            self.last_error = None
        except BaseException as error:
            self.last_error = str(error)
            raise
        finally:
            with self._lock:
                self._home_cancel = None
                if self.state != SystemState.CLOSED:
                    self._state = SystemState.PAUSED

    def pause(self, reason):
        with self._lock:
            if self.state == SystemState.CLOSED:
                return
            if self._home_cancel is not None:
                self._home_cancel.set()
            self._generation += 1
            self._state, self.last_error = SystemState.PAUSED, reason
        # The hand control lock can be busy in an SDK call. Request the arm hold
        # first, and still pause both groups if either stop request fails.
        try:
            self.arms.pause(reason)
        finally:
            self.hands.pause(reason)

    def tick(self, now_monotonic_ns=None):
        if self.state != SystemState.ENGAGED:
            return None
        hand_health = self.hands.health()
        if self.hands.state != SystemState.ENGAGED or not hand_health.ready:
            self.pause(self.hands.last_error or hand_health.detail or "手部遥操作已暂停")
            return None
        result = self.arms.tick(now_monotonic_ns)
        if self.arms.state != SystemState.ENGAGED:
            self.pause(self.arms.last_error or "机械臂遥操作已暂停")
        elif self.hands.state != SystemState.ENGAGED:
            self.pause(self.hands.last_error or "手部遥操作已暂停")
        return result

    def health(self):
        if self.state in (SystemState.DISCONNECTED, SystemState.CLOSED):
            return Health(False, time.monotonic_ns(), self.state.value)
        for runtime in (self.hands, self.arms):
            health = runtime.health()
            if not health.ready:
                return health
        return Health(True, time.monotonic_ns(), "arms and hands ready")

    def status(self, *, include_target=True):
        return {"state": self.state.value, "last_error": self.last_error,
                "health": asdict(self.health()), "mode": "combined",
                "arms": self.arms.status(include_target=include_target),
                "hands": self.hands.status(include_target=include_target)}

    def close(self):
        with self._lock:
            if self.state == SystemState.CLOSED:
                return
            self._state = SystemState.CLOSED
            self._generation += 1
            if self._home_cancel is not None:
                self._home_cancel.set()
        error = None
        try:
            self.arms.close()
        except Exception as problem:
            error = problem
        finally:
            try:
                self.hands.close()
            except Exception as problem:
                if error is not None:
                    raise RuntimeError(f"{error}; hands close failed: {problem}") from error
                raise
        if error is not None:
            raise error
