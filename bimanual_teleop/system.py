"""Lifecycle contract for the combined control runtime."""

from enum import Enum
from typing import Protocol

from bimanual_teleop.types import ControlProfile, Health


class SystemState(str, Enum):
    DISCONNECTED = "disconnected"
    READY = "ready"
    ENGAGED = "engaged"
    PAUSED = "paused"
    FAULT = "fault"
    CLOSED = "closed"


class TeleopSystem(Protocol):
    """Live observation and motion are separate actions.

    Invalid/stale inputs, recentering and critical failures request coordinated
    hold. Resume requires current feedback and a new reference. Concrete device
    pause/enable/stop behavior must be agreed before a runtime is implemented.
    """

    @property
    def state(self) -> SystemState:
        ...

    def start(self) -> None:
        """Connect/acquire only; successful startup reaches READY, not ENGAGED."""
        ...

    def engage(self, profile: ControlProfile) -> None:
        """Verify readiness/configuration and establish a fresh reference."""
        ...

    def pause(self, reason: str) -> None:
        ...

    def resume(self) -> None:
        """Explicitly re-engage after restoring valid inputs; never auto-resume."""
        ...

    def health(self) -> Health:
        ...

    def close(self) -> None:
        """Coordinate shutdown according to verified device behavior."""
        ...
