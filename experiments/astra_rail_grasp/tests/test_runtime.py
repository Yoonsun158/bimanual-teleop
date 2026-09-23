"""Motion and grasp lifecycle exercised with synthetic feedback only."""
from dataclasses import replace
import math
import tempfile
import unittest

from experiments.astra_rail_grasp.fake import FakeBackend
from experiments.astra_rail_grasp.runtime import Runner
from experiments.astra_rail_grasp.tests.test_runtime_edges import Clock, Camera


class RuntimeFlow(unittest.TestCase):
    def setUp(self):
        self.directory = tempfile.TemporaryDirectory()
        self.clock = Clock()
        self.backend = FakeBackend(clock=self.clock)
        self.camera = Camera(self.clock)
        self.runner = Runner(self.backend,self.camera,self.directory.name,
                             allow_motion=True,clock=self.clock)
        self.runner._worker = lambda *args: None
        self.runner.start()
        self.runner.engage(self.visual(onsite_ready=True))
        self.runner.tick(self.clock())

    def tearDown(self):
        if self.runner.state != "closed":
            self.runner.possible_load = False
            self.runner.action = None
            self.runner.shutdown()
        self.directory.cleanup()

    def visual(self, **args):
        self.clock.now += 1
        return {"observation_id":self.runner.observe()["observation_id"],
                "note":"SIMULATED test observation",**args}

    def advance(self, seconds):
        for _ in range(round(seconds/.005)):
            self.clock.now += 5_000_000
            self.runner._send_arms(self.clock())
            for side in self.runner.sides:
                self.runner._send_hand(side,self.clock())
            self.runner.tick(self.clock())

    def hand(self, side, purpose, delta):
        self.runner.hand_step(self.visual(side=side,purpose=purpose,
                              joints_deg={"finger1_joint1":delta}),purpose+side)
        self.advance(.6)
        self.assertIsNone(self.runner.action)

    def test_complete_bimanual_lift_hold_return_release(self):
        for side in self.runner.sides:
            self.runner.mark(self.visual(what="direction-verified",side=side))
        for side in self.runner.sides:
            self.hand(side,"grasp",1)
        self.runner.mark(self.visual(what="grasped"))
        origin = dict(self.backend.poses)
        for i in range(2):
            self.runner.lift(self.visual(mm=10),f"up-{i}")
            self.advance(3.3)
            self.assertIsNone(self.runner.action)
        self.assertAlmostEqual(self.runner.lift_mm,20.)
        self.advance(5.)
        self.assertEqual(self.runner.state,"running")
        self.assertAlmostEqual(self.backend.poses["right"].position_m[1]-origin["right"].position_m[1],.02)
        self.assertAlmostEqual(self.backend.poses["left"].position_m[1]-origin["left"].position_m[1],-.02)
        for i in range(2):
            self.runner.lift(self.visual(mm=-10),f"down-{i}")
            self.advance(3.3)
        self.runner.mark(self.visual(what="supported"))
        for side in self.runner.sides:
            self.hand(side,"release",-1)
        self.runner.mark(self.visual(what="released"))
        self.assertFalse(self.runner.possible_load)
        for side in self.runner.sides:
            self.assertEqual(self.backend.poses[side].position_m,origin[side].position_m)
        self.assertTrue(self.runner.shutdown()["shutdown_confirmed"])

    def test_large_translation_rotation_and_hand_step_rejected(self):
        for moves in ({"right":{"translation_mm":[201,0,0]}},
                      {"right":{"rotation_deg":[0,2,0]}}):
            with self.assertRaises(ValueError):
                self.runner.arm_step(self.visual(moves=moves,near=False),"large")
        with self.assertRaises(ValueError):
            self.runner.hand_step(self.visual(side="right",joints_deg={"finger1_joint1":2}),"large-hand")
        self.assertIsNone(self.runner.action)

    def test_camera_loss_pauses_without_disabling_and_keeps_fixed_target(self):
        self.runner.arm_step(self.visual(moves={"right":{"translation_mm":[0,10,0]}}),"step")
        self.advance(.1)
        self.camera.ready = False
        self.advance(.02)
        self.assertEqual(self.runner.state,"paused")
        frozen = dict(self.backend.poses)
        self.advance(1.)
        self.assertEqual(self.backend.poses,frozen)
        self.assertTrue(self.backend.arm_enabled)
        self.assertTrue(self.backend.hand_enabled)

    def test_sustained_current_stops_further_closure(self):
        self.runner.hand_step(self.visual(side="right",joints_deg={"finger1_joint1":1}),"close")
        self.backend.currents["right"] = (.41,)+(0.,)*19
        self.advance(.25)
        self.assertEqual(self.runner.state,"paused")
        self.assertTrue(self.runner.possible_load)

    def test_single_arm_lag_halts_entire_lift(self):
        for side in self.runner.sides:
            self.runner.mark(self.visual(what="direction-verified",side=side))
        for side in self.runner.sides:
            self.hand(side,"grasp",1)
        self.runner.mark(self.visual(what="grasped"))
        self.backend.freeze_arms.add("left")
        self.runner.lift(self.visual(mm=10),"unequal-lift")
        self.advance(.8)
        self.assertEqual(self.runner.state,"paused")
        self.assertTrue(self.runner.lift_uncertain)
        self.assertIsNone(self.runner.action)

    def test_no_progress_times_out_instead_of_accepting_submission(self):
        self.backend.freeze_arms.add("right")
        self.runner.arm_step(self.visual(moves={"right":{"translation_mm":[0,10,0]}}),"blocked")
        self.advance(7.)
        self.assertEqual(self.runner.state,"paused")
        self.assertIn("timeout",self.runner.reason)

    def test_health_fault_stops_devices(self):
        self.backend.problems = ["right synthetic feedback loss"]
        with self.assertRaises(RuntimeError) as context:
            self.runner.tick(self.clock())
        self.runner.fault(str(context.exception))
        self.assertEqual(self.runner.state,"fault")
        self.assertFalse(self.backend.arm_enabled)
        self.assertFalse(self.backend.hand_enabled)


if __name__ == "__main__":
    unittest.main()
