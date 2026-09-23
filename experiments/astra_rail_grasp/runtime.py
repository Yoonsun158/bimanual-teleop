"""Bounded actions above existing drivers. No SDK is imported by this module."""
from __future__ import annotations

from collections import deque
from copy import deepcopy
from dataclasses import asdict, replace
import json
import math
from pathlib import Path
import queue
import threading
import time
import uuid

import numpy as np
from scipy.spatial.transform import Rotation, Slerp

from bimanual_teleop.types import Pose
from bimanual_teleop.devices.wuji.adapter import JOINT_NAMES, JOINT_LIMITS_RAD

SETTINGS = json.loads(Path(__file__).with_name("settings.json").read_text())


def vector(value, length, name):
    if not isinstance(value, (list, tuple)) or len(value) != length:
        raise ValueError(f"{name} must have {length} values")
    if any(isinstance(x, bool) or not isinstance(x, (int, float)) or not math.isfinite(x) for x in value):
        raise ValueError(f"{name} must contain finite numbers")
    return tuple(float(x) for x in value)


def number(value, name):
    return vector([value], 1, name)[0]


def pose_from(value):
    if value is None:
        raise RuntimeError("actual flange pose unavailable")
    return Pose(value["parent_frame"], value["child_frame"],
                vector(value["position_m"], 3, "position"),
                vector(value["orientation_xyzw"], 4, "quaternion"))


def pose_error(a, b):
    return (float(np.linalg.norm(np.asarray(a.position_m) - b.position_m)),
            float((Rotation.from_quat(a.orientation_xyzw).inv() *
                   Rotation.from_quat(b.orientation_xyzw)).magnitude()))


def blend_pose(a, b, t):
    rotation = Slerp([0., 1.], Rotation.from_quat([a.orientation_xyzw, b.orientation_xyzw]))([t])[0]
    return replace(a, position_m=tuple(x + t * (y-x) for x, y in zip(a.position_m, b.position_m)),
                   orientation_xyzw=tuple(rotation.as_quat()))


class Audit:
    """Disk I/O stays outside the control threads; queue overflow is visible."""
    def __init__(self, path):
        self.path = Path(path)
        self.path.parent.mkdir(parents=True, exist_ok=True)
        self.queue = queue.Queue(maxsize=8192)
        self.error = None
        self.thread = threading.Thread(target=self._write, daemon=True, name="rail-audit")
        self.thread.start()

    def emit(self, event):
        if self.error:
            raise RuntimeError(f"audit failed: {self.error}")
        try:
            self.queue.put_nowait(event)
        except queue.Full as error:
            raise RuntimeError("audit queue overflow") from error

    def _write(self):
        try:
            with self.path.open("a", buffering=1) as stream:
                while True:
                    item = self.queue.get()
                    if item is None:
                        return
                    stream.write(json.dumps(item, ensure_ascii=False, allow_nan=False) + "\n")
        except Exception as error:
            self.error = str(error)

    def close(self):
        self.queue.put(None, timeout=2)
        self.thread.join(timeout=3)


class Runner:
    def __init__(self, backend, camera, run_dir, *, allow_motion=False, clock=time.monotonic_ns):
        self.backend, self.camera = backend, camera
        self.sides = tuple(backend.sides)
        if not self.sides or any(s not in ("left", "right") for s in self.sides):
            raise ValueError("invalid active sides")
        self.run_dir, self.clock = Path(run_dir), clock
        self.run_dir.mkdir(parents=True, exist_ok=True)
        self.audit = Audit(self.run_dir / "events.jsonl")
        self.allow_motion = allow_motion
        self.session_id = uuid.uuid4().hex
        self.state, self.reason = "observe", None
        self.lock = threading.RLock()
        self.dispatch_finished = threading.Condition(self.lock)
        self._dispatch_epoch = 0
        self._dispatches = {}
        self._pause_pending = False
        self.stop_event = threading.Event()
        self.workers = []
        self.feedback = None
        self.arm_targets, self.hand_targets = {}, {}
        self.action = None
        self.history = deque(maxlen=100)
        self.requests = {}
        self.observation = None
        self.last_motion_ns = 0
        self.last_camera_sequences = {}
        self.direction_verified = set()
        self.possible_load = False
        self.stage = "empty"
        self.grip_origin = None
        self.lift_mm = 0.
        self.lift_path = []
        self.lift_uncertain = False
        self.release_sides = set()
        self.travel_mm = 0.
        self._current_since = {}
        self._mismatch_since = None
        self._last_audit_ns = 0
        self._faulting = False

    def log(self, kind, **details):
        self.audit.emit({"event": kind, "monotonic_ns": self.clock(),
                         "session_id": self.session_id, **details})

    def start(self):
        try:
            self.backend.open()
            self.camera.start()
            self.feedback = self.backend.snapshot()
            self.log("opened", sides=self.sides, allow_motion=self.allow_motion, feedback=self.feedback)
            self._worker("monitor", 100, self.tick)
        except BaseException:
            self.camera.close()
            self.backend.close()
            self.audit.close()
            raise

    def _worker(self, name, hz, fn):
        def run():
            deadline = time.monotonic()
            while not self.stop_event.is_set():
                try:
                    fn(self.clock())
                except Exception as error:
                    self.fault(f"{name}: {error}")
                    if name != "monitor":
                        return
                deadline += 1 / hz
                delay = deadline - time.monotonic()
                if delay < 0:
                    deadline = time.monotonic()
                self.stop_event.wait(max(0., delay))
        thread = threading.Thread(target=run, daemon=True, name="rail-" + name)
        self.workers.append(thread)
        thread.start()

    def _targets(self, now):
        arms, hands = dict(self.arm_targets), dict(self.hand_targets)
        action = self.action
        if action is not None:
            fraction = min(1., max(0., (now-action["start_ns"]) / action["duration_ns"]))
            fraction = fraction*fraction*(3-2*fraction)
            if action["kind"] in ("arm-step", "lift"):
                arms = {s: blend_pose(action["arms_start"][s], action["arms_goal"][s], fraction)
                        for s in self.sides}
            else:
                s = action["side"]
                hands[s] = tuple(a + fraction*(b-a) for a, b in
                                 zip(action["hands_start"], action["hands_goal"]))
        return arms, hands

    def _send_arms(self, now):
        with self.lock:
            if self.state not in ("running", "paused", "arming") or not self.arm_targets:
                return
            poses, _ = self._targets(now)
            epoch = self._begin_dispatch()
        try:
            self.backend.send_arms(poses, now)
        finally:
            self._end_dispatch(epoch)

    def _send_hand(self, side, now):
        with self.lock:
            if self.state not in ("running", "paused", "arming") or side not in self.hand_targets:
                return
            _, hands = self._targets(now)
            epoch = self._begin_dispatch()
        try:
            self.backend.send_hand(side, hands[side], now)
        finally:
            self._end_dispatch(epoch)

    def _begin_dispatch(self):
        epoch = self._dispatch_epoch
        self._dispatches[epoch] = self._dispatches.get(epoch, 0) + 1
        return epoch

    def _end_dispatch(self, epoch):
        with self.dispatch_finished:
            self._dispatches[epoch] -= 1
            if not self._dispatches[epoch]:
                del self._dispatches[epoch]
            self.dispatch_finished.notify_all()

    def status(self):
        with self.lock:
            return {"session_id": self.session_id, "state": self.state, "reason": self.reason,
                    "sides": self.sides, "allow_motion": self.allow_motion,
                    "possible_load": self.possible_load, "stage": self.stage,
                    "lift_mm": self.lift_mm, "lift_uncertain": self.lift_uncertain,
                    "travel_mm": self.travel_mm, "settings": SETTINGS,
                    "action": self._action_public(self.action), "recent_actions": list(self.history),
                    "feedback": deepcopy(self.feedback), "camera": self.camera.health()}

    @staticmethod
    def _action_public(action):
        if action is None:
            return None
        return {k: v for k, v in action.items() if k not in
                ("arms_start", "arms_goal", "hands_start", "hands_goal", "settled_since")}

    def observe(self):
        captured = self.camera.snapshot()
        with self.lock:
            now = self.clock()
            cameras = captured.get("cameras", [])
            fresh = [c for c in cameras if c.get("age_s", math.inf) <= .5
                     and c.get("captured_monotonic_ns", 0) > self.last_motion_ns]
            valid = bool(captured.get("ready") and fresh and self.action is None)
            token = uuid.uuid4().hex
            self.observation = {"id": token, "at_ns": now, "valid": valid}
            data = {"observation_id": token, "usable_for_action": valid,
                    "images": captured, "status": self.status()}
            path = self.run_dir / ("observation-" + token + ".json")
        path.write_text(json.dumps(data, ensure_ascii=False, allow_nan=False, indent=2))
        data["path"] = str(path.resolve())
        self.log("observe", observation_id=token, path=data["path"], valid=valid)
        return data

    def _require_observation(self, args, *, consume=False):
        obs = self.observation
        if (not obs or args.get("observation_id") != obs["id"] or not obs["valid"]
                or self.clock()-obs["at_ns"] > SETTINGS["observation_max_age_s"]*1e9):
            raise ValueError("a fresh post-motion observation_id is required")
        if not self.camera.health().get("ready"):
            raise ValueError("camera has no fresh frames")
        if not isinstance(args.get("note"), str) or not args["note"].strip():
            raise ValueError("record the visual judgement in note")
        if consume:
            obs["valid"] = False

    def engage(self, args):
        with self.lock:
            if self.state != "observe" or not self.allow_motion:
                raise ValueError("engage requires an observe session started with --allow-motion")
            self._require_observation(args)
            if args.get("onsite_ready") is not True:
                raise ValueError("onsite_ready must be confirmed for this session")
            feedback = self.backend.snapshot()
            if not feedback["healthy"]:
                raise RuntimeError("devices not ready: " + "; ".join(feedback["problems"]))
            self._require_observation(args, consume=True)
            self.state = "arming"
        try:
            self.backend.configure()
            for side in self.sides:
                seed = self.backend.engage_hand(side)
                with self.lock:
                    if self.state != "arming":
                        raise RuntimeError("engagement interrupted")
                    self.hand_targets[side] = tuple(seed)
                self._worker("hand-" + side, 120, lambda now, s=side: self._send_hand(s, now))
            poses = self.backend.engage_arms()
            with self.lock:
                if self.state != "arming":
                    raise RuntimeError("engagement interrupted")
                self.arm_targets = dict(poses)
                self.state = "running"
                self.last_motion_ns = self.clock()
            self._worker("arms", 200, self._send_arms)
            self.log("engaged", sides=self.sides, arms={s: asdict(p) for s,p in poses.items()},
                     hands=self.hand_targets)
        except Exception as error:
            self.fault(str(error))
            raise
        return self.status()

    def _new_action(self, kind, args, request_id):
        if self.state != "running" or self.action is not None:
            raise ValueError("one action at a time in running state")
        self._require_observation(args)
        if not self.feedback or not self.feedback["healthy"]:
            raise ValueError("feedback is not healthy")
        return {"id": request_id, "kind": kind, "status": "moving", "start_ns": self.clock()}

    def arm_step(self, args, request_id):
        with self.lock:
            action = self._new_action("arm-step", args, request_id)
            if self.possible_load:
                raise ValueError("possible contact/load: use verified lift or mark supported/released")
            moves = args.get("moves")
            if not isinstance(moves, dict) or not moves or not set(moves) <= set(self.sides):
                raise ValueError("moves must name selected sides")
            goals = dict(self.arm_targets)
            duration, distance = .3, 0.
            for side, move in moves.items():
                delta = vector(move.get("translation_mm", [0.,0.,0.]), 3, "translation_mm")
                rotation = vector(move.get("rotation_deg", [0.,0.,0.]), 3, "rotation_deg")
                d, r = float(np.linalg.norm(delta)), float(np.linalg.norm(rotation))
                cap = SETTINGS["near_step_mm"] if args.get("near", True) else SETTINGS["arm_step_mm"]
                if d > cap + 1e-9 or r > SETTINGS["rotation_step_deg"] + 1e-9:
                    raise ValueError("arm step exceeds translation/rotation limit")
                if 1e-9 < d < SETTINGS["translation_min_mm"] - 1e-9:
                    raise ValueError("user requires translations of at least 10 mm")
                p = self.arm_targets[side]
                q = Rotation.from_rotvec(np.radians(rotation)) * Rotation.from_quat(p.orientation_xyzw)
                goals[side] = replace(p, position_m=tuple(a+b/1000 for a,b in zip(p.position_m,delta)),
                                      orientation_xyzw=tuple(q.as_quat()))
                duration = max(duration, 1.5*d/SETTINGS["translation_speed_mm_s"],
                               1.5*r/SETTINGS["rotation_speed_deg_s"])
                distance += d
            if self.travel_mm + distance > SETTINGS["session_travel_budget_mm"]:
                raise ValueError("session cumulative travel budget exhausted")
            action.update(arms_start=dict(self.arm_targets), arms_goal=goals,
                          duration_ns=round(duration*1e9), travel_mm=distance)
            return self._accept(action, args)

    def hand_step(self, args, request_id):
        with self.lock:
            action = self._new_action("hand-step", args, request_id)
            side = args.get("side")
            if side not in self.sides:
                raise ValueError("hand side is not selected")
            purpose = args.get("purpose", "grasp")
            if purpose not in ("prepare", "grasp", "release"):
                raise ValueError("invalid hand purpose")
            if self.stage in ("grasped", "lifted") or (purpose == "release" and self.stage != "supported"):
                raise ValueError("hand motion prohibited until load is supported")
            if purpose == "prepare" and self.possible_load:
                raise ValueError("preparation requires no possible load")
            changes = args.get("joints_deg")
            if not isinstance(changes, dict) or not changes or not set(changes) <= set(JOINT_NAMES):
                raise ValueError("joints_deg must use the named Hand2 joints")
            actual = vector(self.feedback["hands"][side]["position_rad"],20,"hand feedback")
            start = self.hand_targets[side]
            if max(abs(a-b) for a,b in zip(start,actual)) > math.radians(SETTINGS["hand_error_deg"]):
                raise ValueError("hand is not following its previous target")
            goal = list(start)
            changed = False
            for name, value in changes.items():
                delta = number(value, name)
                if abs(delta) > SETTINGS["hand_step_deg"] + 1e-9:
                    raise ValueError("hand step exceeds 1 degree")
                index = JOINT_NAMES.index(name)
                goal[index] += math.radians(delta)
                changed |= abs(delta) > 1e-9
                lo, hi = JOINT_LIMITS_RAD[index]
                if not lo <= goal[index] <= hi:
                    raise ValueError("hand target exceeds mechanical limits")
            if not changed:
                raise ValueError("hand step must change at least one joint")
            action.update(side=side, purpose=purpose, hands_start=start, hands_goal=tuple(goal),
                          duration_ns=round(SETTINGS["hand_transition_s"]*1e9))
            return self._accept(action, args)

    def lift(self, args, request_id):
        with self.lock:
            action = self._new_action("lift", args, request_id)
            if (self.stage not in ("grasped", "lifted") or self.lift_uncertain
                    or set(self.sides) != self.direction_verified):
                raise ValueError("lift requires verified directions, grasp and unambiguous previous motion")
            delta = number(args.get("mm"), "mm")
            if not SETTINGS["translation_min_mm"] - 1e-9 <= abs(delta) <= SETTINGS["lift_step_mm"] + 1e-9:
                raise ValueError("lift step must be 10 mm per the current user instruction")
            target = self.lift_mm + delta
            if target < -1e-9 or target > SETTINGS["lift_total_mm"] + 1e-9:
                raise ValueError("lift must remain within 0..20 mm")
            if delta < 0 and (not self.lift_path or not math.isclose(-delta,self.lift_path[-1],abs_tol=1e-8)):
                raise ValueError("descent must retrace the last completed lift segment")
            goals = {s: replace(self.grip_origin[s], position_m=tuple(
                v + (target/1000 * (-1 if s == "left" else 1) if i == 1 else 0.)
                for i,v in enumerate(self.grip_origin[s].position_m))) for s in self.sides}
            action.update(arms_start=dict(self.arm_targets), arms_goal=goals, delta_mm=delta,
                          duration_ns=round(max(.4,1.5*abs(delta)/SETTINGS["translation_speed_mm_s"])*1e9))
            return self._accept(action, args)

    def _accept(self, action, args):
        self._require_observation(args)
        self.log("action_started", action=self._action_public(action), args=args,
                 arm_goals={s:asdict(p) for s,p in action.get("arms_goal",{}).items()},
                 hand_goal=action.get("hands_goal"))
        self.observation["valid"] = False
        self.travel_mm += action.get("travel_mm", 0.)
        if action["kind"] == "hand-step" and action["purpose"] == "grasp":
            self.possible_load = True
            self.stage = "contact"
            self.release_sides.clear()
        self.action = action
        return {"action_id": action["id"], "status": "moving"}

    def mark(self, args):
        with self.lock:
            if self.state not in ("running", "paused") or self.action is not None:
                raise ValueError("mark requires stationary active session")
            self._require_observation(args)
            what = args.get("what")
            if what == "direction-verified":
                side = args.get("side")
                if side not in self.sides or self.possible_load or self.stage != "empty":
                    raise ValueError("direction verification requires a selected unloaded side")
                self.log("visual_mark", **args)
                self.direction_verified.add(side)
            elif what == "grasped":
                if (self.stage != "contact" or not self.possible_load or self.lift_uncertain
                        or abs(self.lift_mm) > 1e-6 or self.lift_path):
                    raise ValueError("grasped requires prior grasp actions")
                origin = {s:pose_from(self.feedback["arms"][s]["pose"]) for s in self.sides}
                self.log("visual_mark", **args)
                self.grip_origin = origin
                self.possible_load, self.stage = True, "grasped"
                self.lift_mm, self.lift_path = 0., []
                self.release_sides.clear()
            elif what == "supported":
                if (not self.possible_load or self.stage not in ("contact", "grasped", "supported")
                        or self.lift_uncertain or abs(self.lift_mm) > 1e-6 or self.lift_path):
                    raise ValueError("load has not returned along its lift path")
                self.log("visual_mark", **args)
                self.stage = "supported"
                self.release_sides.clear()
            elif what == "released":
                if self.stage != "supported" or self.release_sides != set(self.sides):
                    raise ValueError("support and release steps on all selected hands are required")
                self.log("visual_mark", **args)
                self.possible_load, self.stage = False, "empty"
                self.grip_origin = None
                self.release_sides.clear()
            else:
                raise ValueError("unknown visual mark")
            self.observation["valid"] = False
            return self.status()

    def pause(self, reason="visual pause"):
        with self.lock:
            if self.state not in ("running", "paused"):
                raise ValueError("pause requires active healthy control")
            if self.state == "paused" and not self._pause_pending:
                return self.status()
            if self._pause_pending:
                raise ValueError("pause is awaiting in-flight device submissions")
            arms, hands = self._targets(self.clock())
            if self.feedback and self.feedback["healthy"]:
                arms = {s:pose_from(self.feedback["arms"][s]["pose"]) for s in self.sides}
            self.arm_targets, self.hand_targets = arms, hands
            if self.action:
                self.lift_uncertain |= self.action["kind"] == "lift"
                self.action["status"] = "paused"
                self.history.append(self._action_public(self.action))
            self.action = None
            self.state, self.reason = "paused", reason
            self._pause_pending = True
            old_epoch = self._dispatch_epoch
            self._dispatch_epoch += 1
            self.observation = None
            self.last_motion_ns = self.clock()
            self.log("paused", reason=reason)
            # Condition.wait releases all recursion levels of the RLock, so
            # this is safe even when monitor tick initiated the pause. Arms and
            # hands continue independent hold streams in the new generation.
            drained = self.dispatch_finished.wait_for(
                lambda: not self._dispatches.get(old_epoch), timeout=.1)
            self._pause_pending = False
            if not drained:
                self.fault("device submission did not finish within pause deadline")
                raise RuntimeError("pause could not drain old device submissions")
            return self.status()

    def resume(self, args):
        with self.lock:
            if (self.state != "paused" or self._pause_pending or not self.feedback["healthy"]
                    or self.lift_uncertain):
                raise ValueError("cannot resume faulted/unhealthy or uncertain lift")
            self._require_observation(args)
            for side in self.sides:
                current = vector(self.feedback["hands"][side]["current_a"],20,"hand current")
                if max(map(abs,current)) >= SETTINGS["current_stop_a"]:
                    raise ValueError("current is still above pause threshold")
            self.observation["valid"] = False
            self.state, self.reason = "running", None
            self._current_since.clear()
            self.last_motion_ns = self.clock()
            self.log("resumed")
            return self.status()

    def tick(self, now):
        feedback = self.backend.snapshot()
        with self.lock:
            self.feedback = feedback
            if self.state not in ("running", "paused"):
                return
            if not feedback["healthy"]:
                raise RuntimeError("feedback unhealthy: " + "; ".join(feedback["problems"]))
            if self.audit.error:
                raise RuntimeError("audit storage failed: " + self.audit.error)
            if self.state == "running" and not self.camera.health().get("ready"):
                self.pause("camera stale or unavailable")
            for s in self.sides:
                current = vector(feedback["hands"][s]["current_a"],20,"hand current")
                if max(map(abs,current)) >= SETTINGS["current_stop_a"]:
                    self._current_since.setdefault(s,now)
                    if now-self._current_since[s] >= SETTINGS["current_stop_s"]*1e9 and self.state == "running":
                        self.pause(s + " hand current threshold reached; contact force unknown")
                else:
                    self._current_since.pop(s,None)
            if self.grip_origin and self.stage in ("grasped", "lifted"):
                displacement, bad = [], False
                for s in self.sides:
                    p, origin = pose_from(feedback["arms"][s]["pose"]), self.grip_origin[s]
                    d = np.asarray(p.position_m)-origin.position_m
                    displacement.append(d[1]*(-1 if s == "left" else 1))
                    bad |= math.hypot(d[0],d[2]) > .001 or pose_error(p,origin)[1] > math.radians(.5)
                bad |= len(displacement) == 2 and abs(displacement[0]-displacement[1]) > .001
                if bad:
                    self._mismatch_since = self._mismatch_since or now
                    if now-self._mismatch_since >= .1e9 and self.state == "running":
                        self.pause("lift mismatch/lateral drift/orientation threshold reached")
                else:
                    self._mismatch_since = None
            action = self.action
            if action and now >= action["start_ns"]+action["duration_ns"]:
                if action["kind"] == "hand-step":
                    q = vector(feedback["hands"][action["side"]]["position_rad"],20,"hand position")
                    error = max(abs(a-b) for a,b in zip(q,action["hands_goal"]))
                    arrived = error <= math.radians(.5)
                    if now > action["start_ns"]+action["duration_ns"]+.5e9 and error > math.radians(5):
                        self.pause("hand tracking error exceeds 5 degrees")
                else:
                    arrived = all(pose_error(pose_from(feedback["arms"][s]["pose"]),
                                             action["arms_goal"][s])[0] <= .0005 and
                                  pose_error(pose_from(feedback["arms"][s]["pose"]),
                                             action["arms_goal"][s])[1] <= math.radians(.25)
                                  for s in self.sides)
                moving_sides = [action["side"]] if action["kind"] == "hand-step" else self.sides
                group = "hands" if action["kind"] == "hand-step" else "arms"
                n = 20 if group == "hands" else 7
                arrived &= all(max(map(abs,vector(feedback[group][s]["velocity_rad_s"],n,"velocity")))
                               <= math.radians(.5) for s in moving_sides)
                if self.action is action and arrived:
                    action.setdefault("settled_since",now)
                    if now-action["settled_since"] >= .2e9:
                        self._complete(action,now)
                else:
                    action.pop("settled_since",None)
                if self.action is action and now > action["start_ns"]+action["duration_ns"]+3e9:
                    self.pause("action did not reach its measured target before timeout")
            if now-self._last_audit_ns >= .1e9:
                self._last_audit_ns = now
                self.log("feedback", state=self.state, feedback=feedback)

    def _complete(self, action, now):
        if action["kind"] == "hand-step":
            self.hand_targets[action["side"]] = action["hands_goal"]
            if action["purpose"] == "release":
                self.release_sides.add(action["side"])
        else:
            self.arm_targets = dict(action["arms_goal"])
        if action["kind"] == "lift":
            self.lift_mm += action["delta_mm"]
            if action["delta_mm"] > 0:
                self.lift_path.append(action["delta_mm"])
            else:
                self.lift_path.pop()
            self.stage = "lifted" if self.lift_mm > 1e-8 else "grasped"
        action.update(status="completed", completed_ns=now)
        self.history.append(self._action_public(action))
        self.log("action_completed", action=self._action_public(action))
        self.action = None
        self.last_motion_ns = now

    def fault(self, reason):
        with self.lock:
            if self._faulting or self.state in ("fault", "closed"):
                return
            self._faulting = True
            self.state, self.reason = "fault", reason
            if self.action:
                self.action["status"] = "fault"
                self.history.append(self._action_public(self.action))
                self.action = None
            self.observation = None
        try:
            self.backend.fault_stop(reason)
        except Exception as error:
            self.reason += "; stop unconfirmed: " + str(error)
        finally:
            self._faulting = False
            try:
                self.log("fault", reason=self.reason, possible_load=self.possible_load,
                         warning="holding is not guaranteed; onsite intervention required")
            except Exception:
                pass

    def shutdown(self):
        with self.lock:
            if self.possible_load:
                raise ValueError("possibly loaded: support and release object before normal shutdown")
            if self.action is not None or self.state == "arming":
                raise ValueError("pause current action before shutdown")
            self.state = "closing"
            self.stop_event.set()
        for worker in self.workers:
            if worker is not threading.current_thread():
                worker.join(timeout=2)
        errors = []
        for obj in (self.backend,self.camera):
            try:
                obj.close()
            except Exception as error:
                errors.append(str(error))
        self.state = "closed"
        self.log("closed", errors=errors)
        self.audit.close()
        if errors:
            raise RuntimeError("shutdown unconfirmed: " + "; ".join(errors))
        return {"state":"closed", "shutdown_confirmed":True}

    def handle(self, request):
        op, args = request.get("op"), request.get("args", {})
        if not isinstance(args,dict):
            raise ValueError("args must be an object")
        if op == "status":
            return self.status()
        if op == "observe":
            return self.observe()
        if request.get("session_id") != self.session_id:
            raise ValueError("session_id mismatch; stale/reconnected commands are not replayed")
        rid = request.get("id")
        if not isinstance(rid,str) or not 1 <= len(rid) <= 128:
            raise ValueError("mutation requires a bounded unique id")
        fingerprint = json.dumps([op,args],sort_keys=True,allow_nan=False)
        with self.lock:
            entry = self.requests.get(rid)
            if entry is not None:
                if entry["fingerprint"] != fingerprint:
                    raise ValueError("request id reused with different content")
                duplicate = True
            else:
                expiry = number(request.get("expires_ns"),"expires_ns")
                now = self.clock()
                if not now < expiry <= now+60e9:
                    raise ValueError("command expired or deadline too far in future")
                if len(self.requests) >= 10000:
                    raise ValueError("session command budget exhausted")
                entry = {"fingerprint": fingerprint, "done": threading.Event()}
                self.requests[rid] = entry
                duplicate = False
        if duplicate:
            if not entry["done"].wait(timeout=15):
                raise ValueError("original request is still running; retry the same id")
            if "error" in entry:
                raise ValueError(entry["error"])
            return deepcopy(entry["result"])
        try:
            if op == "engage": result = self.engage(args)
            elif op == "arm-step": result = self.arm_step(args,rid)
            elif op == "hand-step": result = self.hand_step(args,rid)
            elif op == "lift": result = self.lift(args,rid)
            elif op == "mark": result = self.mark(args)
            elif op == "pause": result = self.pause(args.get("reason","operator visual pause"))
            elif op == "resume": result = self.resume(args)
            elif op == "shutdown": result = self.shutdown()
            else: raise ValueError("unknown operation")
        except Exception as error:
            entry["error"] = str(error)
            entry["done"].set()
            raise
        entry["result"] = deepcopy(result)
        entry["done"].set()
        return result
