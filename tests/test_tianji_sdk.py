"""Exercise the official Python boundary without opening a network connection."""

import argparse
import ctypes as ct
import hashlib
import json
from pathlib import Path
import shutil
import tempfile
import threading
import time
import unittest
from unittest.mock import Mock, patch

from bimanual_teleop.devices.tianji import sdk
from bimanual_teleop.devices.tianji.driver import TianjiDriver
from bimanual_teleop.devices.tianji.model import MotionProfile
from bimanual_teleop.types import CommandEvent, CommandStatus
from tests.support.tianji import TianjiFixture


class OfficialSDKTests(unittest.TestCase):
    def test_vendor_files_match_distribution_manifest(self):
        manifest = json.loads((sdk.DEFAULT_SDK_ROOT / "manifest.json").read_text())
        self.assertEqual(manifest["commit"], sdk.SDK_COMMIT)
        for name, digest in manifest["files"].items():
            with self.subTest(name=name):
                self.assertEqual(hashlib.sha256((sdk.DEFAULT_SDK_ROOT / name).read_bytes()).hexdigest(), digest)

    def test_missing_and_modified_components_fail_before_loading_library(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            with patch("ctypes.CDLL", side_effect=AssertionError("must not load")):
                with self.assertRaisesRegex(RuntimeError, "file missing"):
                    sdk.load_sdk("kine", root)
                shutil.copytree(sdk.DEFAULT_SDK_ROOT / "SDK_PYTHON", root / "SDK_PYTHON")
                (root / "SDK_PYTHON/fx_kine.py").write_text("# changed\n")
                with self.assertRaisesRegex(RuntimeError, "modified"):
                    sdk.load_sdk("kine", root)

    def test_unsupported_architecture_fails_before_load(self):
        with tempfile.TemporaryDirectory() as directory, patch.object(sdk.platform, "machine", return_value="aarch64"):
            with self.assertRaisesRegex(RuntimeError, "Linux x86_64"):
                sdk.load_sdk("robot", directory)

    def test_official_structures_and_key_offsets(self):
        robot, kine = sdk.load_sdk("robot"), sdk.load_sdk("kine")
        self.assertEqual(ct.sizeof(robot.DCSS), 1428)
        self.assertEqual(robot.DCSS.m_In.offset, 24)
        self.assertEqual(robot.DCSS.m_Out.offset, 760)
        self.assertEqual(ct.sizeof(kine.FX_InvKineSolvePara), 992)
        self.assertEqual(ct.sizeof(ct.c_long), 8)

    def test_constructing_control_adapter_never_connects(self):
        module = sdk.load_sdk("robot")
        with patch.object(module.Marvin_Robot, "connect", side_effect=AssertionError("must not connect")):
            control = sdk.ControlSDK()
            self.assertFalse(control._opened)

    def test_long_parameter_buffer_preserves_64_bit_signed_values(self):
        control = sdk.ControlSDK()
        control._opened = True  # Substitute only the official C function, no connect.
        function_type = ct.CFUNCTYPE(ct.c_long, ct.c_char_p, ct.POINTER(ct.c_long))
        for value in (2**40 + 7, -(2**40) + 5):
            @function_type
            def getter(name, output):
                output[0] = value
                return 0
            with patch.object(control.robot.robot, "OnGetIntPara", getter):
                self.assertEqual(control.get_int("SERVO0ERR0"), value)
        @function_type
        def rejected(name, output):
            return 1
        with patch.object(control.robot.robot, "OnGetIntPara", rejected):
            with self.assertRaisesRegex(RuntimeError, "returned 1"):
                control.get_int("VERSION")

    def test_old_library_flag_has_actionable_error(self):
        parser = argparse.ArgumentParser()
        sdk.add_sdk_argument(parser)
        with self.assertRaises(SystemExit), patch.object(parser, "_print_message") as output:
            parser.parse_args(["--library", "old.so"])
        self.assertIn("--sdk-root", "".join(str(call) for call in output.call_args_list))
        self.assertIsNone(parser.parse_args([]).sdk_root)


class ControlCommandsTests(unittest.TestCase):
    def setUp(self):
        # A fake official Marvin_Robot, below the production adapter boundary.
        self.robot = Mock()
        self.control = sdk.ControlSDK.__new__(sdk.ControlSDK)
        self.control.robot = self.robot
        self.control.module = sdk.load_sdk("robot")
        self.control._opened = True
        self.control._lock = threading.RLock()
        self.control._sequence = (0, 0)
        self.control._index = 0
        self.control.cancelled = lambda: False

    def target(self):
        return self.control.submit(3, list(range(14)), time.monotonic_ns() + 50_000_000)

    def test_both_arms_share_one_official_submission(self):
        observed = self.target()
        self.assertLessEqual(observed, time.monotonic_ns())
        self.robot.clear_set.assert_called_once_with()
        self.robot.send_cmd.assert_called_once_with()
        self.assertEqual(self.robot.set_joint_cmd_pose.call_args_list,
                         [unittest.mock.call("A", list(range(7))), unittest.mock.call("B", list(range(7, 14)))])

    def test_failed_second_arm_discards_entire_unsubmitted_build(self):
        self.robot.set_joint_cmd_pose.side_effect = [True, False]
        with self.assertRaisesRegex(RuntimeError, "target"):
            self.target()
        self.robot.send_cmd.assert_not_called()
        self.assertEqual(self.robot.clear_set.call_count, 2)

    def test_expired_target_is_rejected_before_build(self):
        with self.assertRaisesRegex(RuntimeError, "expired"):
            self.control.submit(1, [0.]*14, time.monotonic_ns()-1)
        self.robot.clear_set.assert_not_called()

    def test_expiry_during_build_prevents_submission(self):
        now = [1_000_000_000]
        self.robot.set_joint_cmd_pose.side_effect = lambda *args: now.__setitem__(0, 1_010_000_000) or True
        with patch.object(sdk.time, "monotonic_ns", side_effect=lambda: now[0]):
            with self.assertRaisesRegex(RuntimeError, "expired"):
                self.control.submit(1, [0.]*14, 1_005_000_000)
        self.robot.send_cmd.assert_not_called()

    def test_cancellation_during_build_prevents_submission(self):
        cancelled = threading.Event()
        self.control.cancelled = cancelled.is_set
        self.robot.set_joint_cmd_pose.side_effect = lambda *args: cancelled.set() or True
        with self.assertRaisesRegex(RuntimeError, "cancelled"):
            self.target()
        self.robot.send_cmd.assert_not_called()

    def test_busy_slot_wait_is_bounded_without_constructing_targets(self):
        now = [1_000_000_000]
        self.robot.clear_set.return_value = False
        with patch.object(sdk.time, "monotonic_ns", side_effect=lambda: now[0]), \
                patch.object(sdk.time, "sleep", side_effect=lambda duration: now.__setitem__(0, now[0] + 1_000_000)):
            with self.assertRaisesRegex(RuntimeError, "pending"):
                self.control.submit(1, [0.]*14, 1_050_000_000)
        self.assertEqual(now[0], 1_005_000_000)
        self.robot.set_joint_cmd_pose.assert_not_called()

    def test_send_api_rejection_is_not_reported_as_success(self):
        self.robot.send_cmd.return_value = False
        with self.assertRaisesRegex(RuntimeError, "submission"):
            self.target()

    def test_disable_ignores_motion_cancellation_and_never_sets_a_target(self):
        self.control.cancelled = lambda: True
        self.control.disable(2)
        self.robot.set_state.assert_called_once_with("B", 0)
        self.robot.set_joint_cmd_pose.assert_not_called()

    def test_configure_preserves_every_profile_parameter(self):
        profile = MotionProfile.from_control_profile(TianjiFixture.make_profile(("right",)))
        p = profile.arms["right"]
        self.control.configure(2, [None, p])
        self.robot.set_tool.assert_called_once_with("B", [0.]*6, list(p.tool_dyn10))
        self.robot.set_vel_acc.assert_called_once_with("B", 100, 100)
        self.robot.set_cart_kd_params.assert_called_once_with("B", [1.]*7, [.5]*7)
        self.robot.set_EefCart_control_params.assert_called_once_with("B", 1, [0.]*7)
        self.robot.set_PD_vel_est_step.assert_called_once_with("B", 0)

    def test_clear_and_stop_use_checked_official_parameters_without_enable(self):
        self.robot.robot.OnSetIntPara.return_value = 0
        self.control.clear_errors(1)
        self.control.hold(3)
        names = [call.args[0].rstrip(b"\0") for call in self.robot.robot.OnSetIntPara.call_args_list]
        self.assertEqual(names, [b"RESET1", b"RSTA01"])
        self.robot.set_state.assert_not_called()
        self.robot.robot.OnSetIntPara.return_value = 1
        with self.assertRaisesRegex(RuntimeError, "RESET0 returned 1"):
            self.control.clear_errors(0)

    def test_raw_feedback_preserves_float32_precision_and_duplicates_do_not_refresh_time(self):
        value = self.control.module.DCSS()
        value.m_Out[0].m_OutFrameSerial = 1
        value.m_Out[0].m_FB_Joint_Pos[0] = 1.2345678
        value.m_Out[0].m_LowSpdFlag = b"\1"
        def read(pointer):
            ct.memmove(pointer, ct.byref(value), ct.sizeof(value))
            return True
        self.robot.robot.OnGetBuf.side_effect = read
        first = self.control.poll_feedback()
        self.assertEqual(first.q[0], value.m_Out[0].m_FB_Joint_Pos[0])
        self.assertNotEqual(first.q[0], round(first.q[0], 4))
        self.assertEqual(first.low_speed[0], 1)
        self.assertIsNone(self.control.poll_feedback())
        value.m_Out[1].m_OutFrameSerial = 1
        second = self.control.poll_feedback()
        self.assertGreater(second.received_ns, first.received_ns)
        self.assertEqual(second.sequence, [1, 1])

    def test_process_owner_guard_does_not_release_other_connection(self):
        self.control._opened = False
        owner = object()
        with patch.object(sdk, "_OWNER", owner):
            with self.assertRaisesRegex(RuntimeError, "owner"):
                self.control.open("192.0.2.1")
            self.assertIs(sdk._OWNER, owner)
        self.robot.connect.assert_not_called()
        self.robot.release_robot.assert_not_called()


class SubmissionMeaningTests(unittest.TestCase, TianjiFixture):
    def setUp(self):
        TianjiFixture.__init__(self)
        self.addCleanup(self.driver.close)

    def test_sdk_acceptance_never_emits_a_udp_send_receipt(self):
        self.engage()
        command = self.command()
        self.assertTrue(self.driver.submit(command).accepted)
        statuses = [e.status for e in self.sink.events if isinstance(e, CommandEvent) and e.command_id == command.command_id]
        self.assertEqual(statuses, [CommandStatus.ACCEPTED, CommandStatus.SDK_SUBMITTED])
        self.assertIn("unavailable", self.driver.metadata["udp_send_receipt"])
        self.assertFalse(hasattr(self.driver, "_on_send"))

    def test_sdk_rejection_cannot_complete_move_from_matching_feedback(self):
        self.configure()
        self.native.send_joint_move = False
        with self.assertRaisesRegex(RuntimeError, "rejected"):
            self.driver.move_joints("left", self.driver.get_latest().payload.arms["left"].joints.position_rad)
        self.assertFalse(self.events("tianji.joint_move_completed"))

    def test_configuration_requires_controller_echo_before_enabling(self):
        self.native.echo = False
        self.driver.engagement_timeout_ns = 5_000_000
        with self.assertRaisesRegex(RuntimeError, "configuration"):
            self.configure()
        self.assertIsNone(self.driver.profile)
        self.assertFalse(any(name == "engage" for name, _ in self.native.calls))
