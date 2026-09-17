"""Real SDK through Quest mapping and resampling, without opening any device.

The seeds are fixed in advance, not selected for passing after a solver change.
All eight seeds 0..7 of this gentle motion completed the ordinary SDK baseline
in the independent four-second audit. This regression uses seeds 0, 2, and 6
to keep the suite small, and checks the target against the ordinary SDK on
every commanded frame, including delayed ticks and submission overhead.
"""

import math
import random
from types import SimpleNamespace
import unittest
from unittest.mock import patch

from tests.support.geometry import orientation_distance
from bimanual_teleop.control.arm.mapping import (
    _from_matrix, matmul, rotation_matrix,
)
from bimanual_teleop.devices.tianji.model import TianjiKinematics
from bimanual_teleop.system import SystemState
from tests.support.quest import RuntimeFixture


READY_DEG = {"left": (30, -60, -34, -52, 30, 12, 4),
             "right": (-30, -60, 34, -52, -30, 12, -4)}
INITIAL_POSITION = {"left": (0., .2, 1.), "right": (0., -.2, 1.)}
INITIAL_ROTATION = {"left": (-.5, -.5, .5, .5), "right": (.5, -.5, -.5, .5)}


def operator_frames(seed, duration_s):
    """90 Hz six-axis gestures with reproducible sub-mm/0.03-degree noise."""
    rng = random.Random(seed)
    components = {side: [[(rng.uniform(.2, .9), rng.uniform(-math.pi, math.pi),
                          rng.uniform(.4, 1.) * (1 if term == 0 else .35))
                         for term in range(2)] for _ in range(6)]
                  for side in ("left", "right")}
    for frame in range(1, round(duration_s * 90) + 1):
        seconds = frame / 90
        positions, rotations = {}, {}
        for side in ("left", "right"):
            values = [sum(amplitude * (math.sin(2 * math.pi * frequency * seconds + phase)
                                       - math.sin(phase)) for frequency, phase, amplitude in axis)
                      for axis in components[side]]
            translation = [value * .025 + rng.uniform(-.00015, .00015) for value in values[:3]]
            rotation = [value * .1 + rng.uniform(-.0005, .0005) for value in values[3:]]
            positions[side] = tuple(a + b for a, b in zip(INITIAL_POSITION[side], translation))
            angle = math.sqrt(sum(value * value for value in rotation))
            quaternion = (tuple(value / angle * math.sin(angle / 2) for value in rotation)
                          + (math.cos(angle / 2),)) if angle else (0., 0., 0., 1.)
            rotations[side] = _from_matrix(matmul(rotation_matrix(quaternion),
                                                rotation_matrix(INITIAL_ROTATION[side])))
        yield round(seconds * 1e9), positions, rotations


class TianjiTimingRegressionTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.kinematics = TianjiKinematics()

    def test_six_axis_source_survives_delayed_ticks_and_submit_overhead(self):
        for seed in (0, 2, 6):
            for submit_cost_ns in (0, 3_000_000):
                with self.subTest(seed=seed, submit_cost_ns=submit_cost_ns):
                    self.run_motion(seed, submit_cost_ns)

    def run_motion(self, seed, submit_cost_ns):
        fx = RuntimeFixture(coordinate_frame="world")
        self.addCleanup(fx.runtime.close)
        fx.driver.q = {side: tuple(math.radians(value) for value in degrees)
                       for side, degrees in READY_DEG.items()}
        fx.driver.emit()
        fx.kine = fx.executor.kinematics = self.kinematics
        fx.parsed.model = self.kinematics.model
        fx.parsed.arms = {side: SimpleNamespace(velocity_ratio=100, acceleration_ratio=100)
                          for side in ("left", "right")}
        submit = fx.driver.submit

        def delayed_submit(command):
            result = submit(command)
            # Keep target time independent from time spent completing submit.
            fx.clock.advance(submit_cost_ns)
            return result

        fx.driver.submit = delayed_submit
        origin = fx.clock()
        frames = iter(operator_frames(seed, 4.))
        pending = next(frames, None)
        tick, cycles = origin, 0
        intervals = set()
        previous_v = {s: (0.,) * 7 for s in ("left", "right")}
        with patch("bimanual_teleop.control.arm.cartesian.time.monotonic_ns", side_effect=fx.clock):
            fx.runtime.engage()
            while tick < origin + 4_000_000_000:
                cycles += 1
                interval_ns = (20_000_000 if cycles % 37 == 0 else
                               10_000_000 if cycles % 17 == 0 else 5_000_000)
                intervals.add(interval_ns)
                tick += interval_ns
                fx.clock.now = tick
                # Source capture times remain at 90 Hz even when a delayed
                # command tick drains multiple already-arrived source frames.
                while pending is not None and origin + pending[0] <= tick:
                    source_ns, positions, rotations = pending
                    fx.quest.emit(positions=positions, rotations=rotations,
                                  query_ns=origin + source_ns - 500_000_000)
                    pending = next(frames, None)
                fx.driver.emit()
                previous = dict(fx.driver.q)
                target = fx.runtime.tick()
                self.assertIsNotNone(target, fx.runtime.last_error)
                command = fx.driver.commands[-1]
                self.assertEqual(command.payload.cartesian_targets, target.tool_poses)
                for side in ("left", "right"):
                    dt = .005 if cycles == 1 else interval_ns / 1e9
                    v = tuple((a-b)/dt for a, b in zip(command.payload.targets[side], previous[side]))
                    model = self.kinematics.model.arm(side)
                    for value, old, row in zip(v, previous_v[side], model.limits_native):
                        self.assertLessEqual(abs(value), math.radians(row[2]) + 1e-6)
                        self.assertLessEqual(abs(value-old), math.radians(row[3])*dt + 1e-6)
                    previous_v[side] = v
                    actual = self.kinematics.fk(side, command.payload.targets[side])
                    self.assertLess(math.dist(actual.position_m, target.tool_poses[side].position_m), 1e-6)
                    self.assertLess(orientation_distance(actual.orientation_xyzw,
                                                        target.tool_poses[side].orientation_xyzw), 1e-6)
            self.assertEqual(intervals, {5_000_000, 10_000_000, 20_000_000})
            self.assertGreater(cycles, 500)
            self.assertEqual(fx.runtime.state, SystemState.ENGAGED)
            self.assertEqual(fx.driver.holds, [])
            fx.runtime.close()


if __name__ == "__main__":
    unittest.main()
