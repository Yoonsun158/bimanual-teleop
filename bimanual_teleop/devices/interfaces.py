"""Vendor SDKs belong behind these contracts, never in shared types."""

from typing import Protocol, TypeVar

from bimanual_teleop.types import (
    CommandEvent,
    ControlProfile,
    DeviceCommand,
    Event,
    Health,
    Sample,
    Submission,
)

StateT = TypeVar("StateT", covariant=True)
CommandT = TypeVar("CommandT", contravariant=True)


class RealtimeObserver(Protocol):
    """Nonblocking in-memory callback for live samples and status events.

    Returning False or raising means the observer cannot safely process live
    state. Sources latch that failure, and an engaged driver requests hold.
    """

    def try_publish(self, sample: Sample[object]) -> bool:
        ...

    def try_event(self, event: Event | CommandEvent) -> bool:
        ...


class SampleSource(Protocol[StateT]):
    """Single owner of a source; raw publication is separate from latest reads."""

    def start(self, sink: RealtimeObserver | None = None) -> None:
        """Begin live reception without enabling motion; bind observer first."""
        ...

    def get_latest(self) -> StateT | None:
        """Nonblocking snapshot; rereading MUST preserve original sample times."""
        ...

    def health(self) -> Health:
        ...

    def close(self) -> None:
        """Release source resources, including after partial startup."""
        ...


class RobotDriver(SampleSource[StateT], Protocol[StateT, CommandT]):
    """Own one hardware connection; a Tianji driver owns both arms together."""

    def configure(self, profile: ControlProfile) -> None:
        """Apply/verify configuration while paused; do not engage movement."""
        ...

    def engage(self) -> None:
        """Explicit activation after validation; takeover is device-specific."""
        ...

    def submit(self, command: DeviceCommand[CommandT]) -> Submission:
        """Accept without implicitly engaging; report later sends to the observer."""
        ...

    def request_hold(self, reason: str) -> None:
        """Request the device-specific pause behavior; not a physical-stop ack."""
        ...
