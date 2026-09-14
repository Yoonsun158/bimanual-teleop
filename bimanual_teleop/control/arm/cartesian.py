"""Flange targets resolved by official IK and submitted as one arm group."""

from __future__ import annotations

import math
import time

from bimanual_teleop.devices.tianji.driver import TianjiDriver, TianjiFrame, TianjiJointCommand
from bimanual_teleop.devices.tianji.model import TianjiKinematics
from bimanual_teleop.types import ControlProfile, DeviceCommand, Pose, RobotTarget, Sample, SampleRef, Side, Submission


CONTROL_HZ = 200
PERIOD_NS = 1_000_000_000 // CONTROL_HZ


class TianjiCartesianExecutor:
    """Synchronous IK + validation; the caller supplies fresh targets at 200 Hz.

    No target queue, interpolation, or background retransmission is introduced.
    An accepted target becomes the next IK reference. Re-engagement always uses
    measured joints, including after a driver watchdog hold.
    """

    def __init__(self, driver: TianjiDriver, kinematics: TianjiKinematics) -> None:
        self.driver = driver
        self.kinematics = kinematics
        self._reference: dict[Side, tuple[float, ...]] = {}
        self._engagement_poses: dict[Side, Pose] = {}
        self._engagement_ref: SampleRef | None = None

    @property
    def engagement_poses(self) -> dict[Side, Pose]:
        """Origins from the exact sample used by the driver's engagement."""
        return dict(self._engagement_poses)

    @property
    def engagement_ref(self) -> SampleRef | None:
        return self._engagement_ref

    def configure(self, profile: ControlProfile) -> None:
        self.driver.configure(profile)
        self._reference.clear()
        self._engagement_poses.clear()
        self._engagement_ref = None

    def _resolve_reference(self, sample: Sample[TianjiFrame]) -> tuple[dict, dict]:
        reference, poses = {}, {}
        for side in self.driver.profile.active_arms:
            joints = sample.payload.arms[side].joints.position_rad
            if len(joints) != 7 or any(q is None or not math.isfinite(q) for q in joints):
                raise ValueError(f"{side}: actual joints are invalid")
            reference[side] = tuple(float(q) for q in joints)
            poses[side] = self.kinematics.fk(side, reference[side])
            # A singular configuration cannot initialize Cartesian following.
            self.kinematics.ik(side, poses[side], reference[side])
        return reference, poses

    def engage(self) -> None:
        profile = self.driver.profile
        sample = self.driver.get_latest()
        if profile is None or sample is None:
            raise RuntimeError("configure and receive actual joints before engagement")
        if not self.driver.health().ready:
            raise RuntimeError("feedback is not ready for engagement")
        self._resolve_reference(sample)
        self.driver.engage()
        try:
            sample = self.driver.engagement_sample
            if sample is None:
                raise RuntimeError("driver did not retain its engagement sample")
            reference, poses = self._resolve_reference(sample)
        except (RuntimeError, ValueError):
            self.request_hold("driver engagement reference could not initialize IK")
            raise
        self._reference = reference
        self._engagement_poses = dict(poses)
        self._engagement_ref = sample.header.ref

    def _reject(self, target: RobotTarget, reason: str) -> Submission:
        self.request_hold(f"Cartesian target {target.command_id}: {reason}")
        return Submission(target.command_id, False, reason)

    def submit(self, target: RobotTarget, *, before_submit=None) -> Submission:
        profile = self.driver.profile
        now = time.monotonic_ns()
        if not self.driver.engaged or not self._reference or profile is None:
            return Submission(target.command_id, False, "executor is not engaged")
        if target.control_profile_id != profile.profile_id:
            return self._reject(target, "control profile does not match")
        if not target.created_monotonic_ns <= now < target.expires_monotonic_ns:
            return self._reject(target, "target is expired or created in the future")
        if target.hand_joints or set(target.tool_poses) != set(profile.active_arms):
            return self._reject(target, "target must contain exactly the configured arms and no hands")
        poses = dict(target.tool_poses)
        try:
            targets = {side: self.kinematics.ik(side, poses[side], self._reference[side])
                       for side in profile.active_arms}
            if before_submit is not None:
                before_submit()
        except (ValueError, RuntimeError) as error:
            return self._reject(target, str(error))
        if time.monotonic_ns() >= target.expires_monotonic_ns:
            return self._reject(target, "target expired during IK")
        command = DeviceCommand(
            device_id="tianji", command_id=target.command_id,
            payload=TianjiJointCommand(targets=targets, cartesian_targets=poses),
            source_refs=tuple(target.source_refs), created_monotonic_ns=target.created_monotonic_ns,
            expires_monotonic_ns=target.expires_monotonic_ns,
            control_profile_id=target.control_profile_id,
        )
        result = self.driver.submit(command)
        if result.accepted:
            self._reference = targets
        else:
            self.request_hold(f"Cartesian target {target.command_id}: {result.reason}")
        return result

    def request_hold(self, reason: str) -> None:
        self._reference.clear()
        self.driver.request_hold(reason)
