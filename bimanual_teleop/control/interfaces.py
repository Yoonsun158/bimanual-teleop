"""High-level mapping and execution boundaries, separate from vendor transport."""

from typing import Protocol

from bimanual_teleop.types import (
    ControlProfile,
    OperatorInput,
    RobotState,
    RobotTarget,
    Submission,
)


class TeleopMapper(Protocol):
    def reset_reference(self, operator: OperatorInput, robot: RobotState) -> None:
        """Re-engage against current valid poses; do not reuse an old origin."""
        ...

    def compute(
        self,
        operator: OperatorInput,
        robot: RobotState,
        *,
        now_monotonic_ns: int,
    ) -> RobotTarget:
        """Map inputs into tool/hand goals, preserving their source references."""
        ...


class RobotExecutor(Protocol):
    """Resolve targets with IK/constraints and coordinate device submissions.

    The executor uses device kinematics and applies motion constraints; this
    interface does not introduce a second low-level motor controller.
    """

    def configure(self, profile: ControlProfile) -> None:
        ...

    def engage(self) -> None:
        """Coordinate explicit device activation after configuration checks."""
        ...

    def submit(self, target: RobotTarget) -> Submission:
        """Bounded local acceptance, not a synchronized hardware-execution ack."""
        ...

    def request_hold(self, reason: str) -> None:
        """Coordinate all configured arms and hands using verified behavior."""
        ...
