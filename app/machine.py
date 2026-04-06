from __future__ import annotations

from dataclasses import dataclass, field
from math import sqrt
from typing import Dict, List, Tuple


@dataclass(slots=True)
class MotionLimits:
    x_min: float = -220.0
    x_max: float = 220.0
    y_min: float = -220.0
    y_max: float = 220.0
    z_min: float = -250.0
    z_max: float = 250.0


@dataclass(slots=True)
class MoveResult:
    start: Dict[str, float]
    end: Dict[str, float]
    duration_s: float
    motion: dict | None = None


@dataclass(slots=True)
class HomeSequenceResult:
    start: Dict[str, float]
    end: Dict[str, float]
    duration_s: float
    motions: List[dict]


@dataclass
class PrinterState:
    limits: MotionLimits = field(default_factory=MotionLimits)
    position: Dict[str, float] = field(
        default_factory=lambda: {"X": 0.0, "Y": 0.0, "Z": 0.0, "A": 0.0}
    )
    absolute_mode: bool = True
    motors_enabled: bool = True
    soft_endstops: bool = True
    feedrate_mm_min: float = 1200.0
    rotary_speed_deg_s: float = 45.0
    max_feedrate_mm_s: Dict[str, float] = field(
        # "Normal" hobby-printer defaults: XY fast, Z conservative, A moderate.
        default_factory=lambda: {"X": 300.0, "Y": 300.0, "Z": 12.0, "A": 90.0}
    )
    acceleration: Dict[str, float] = field(
        default_factory=lambda: {"X": 1000.0, "Y": 1000.0, "Z": 100.0, "A": 250.0}
    )
    homing_feedrate: Dict[str, float] = field(
        default_factory=lambda: {"X": 50.0, "Y": 50.0, "Z": 4.0, "A": 20.0}
    )
    path: List[Tuple[float, float, float]] = field(default_factory=list)

    def snapshot(self) -> dict:
        return {
            "position": {k: round(v, 4) for k, v in self.position.items()},
            "absolute_mode": self.absolute_mode,
            "motors_enabled": self.motors_enabled,
            "soft_endstops": self.soft_endstops,
            "feedrate_mm_min": self.feedrate_mm_min,
            "rotary_speed_deg_s": self.rotary_speed_deg_s,
            "max_feedrate_mm_s": {k: round(v, 3) for k, v in self.max_feedrate_mm_s.items()},
            "acceleration": {k: round(v, 3) for k, v in self.acceleration.items()},
            "homing_feedrate": {k: round(v, 3) for k, v in self.homing_feedrate.items()},
            "limits": {
                "x": [self.limits.x_min, self.limits.x_max],
                "y": [self.limits.y_min, self.limits.y_max],
                "z": [self.limits.z_min, self.limits.z_max],
            },
            "motion_model": "trapezoid",
            "path": [
                {"x": p[0], "y": p[1], "z": p[2]} for p in self.path[-5000:]
            ],
        }

    @staticmethod
    def _trapezoid_time(distance: float, target_speed: float, accel: float) -> float:
        if distance <= 1e-9:
            return 0.0

        target_speed = max(1e-6, target_speed)
        accel = max(1e-6, accel)

        accel_dist = (target_speed * target_speed) / (2.0 * accel)
        if 2.0 * accel_dist <= distance:
            cruise_dist = distance - (2.0 * accel_dist)
            return (2.0 * target_speed / accel) + (cruise_dist / target_speed)

        # Move is too short to hit cruise speed: triangular profile.
        peak_speed = sqrt(distance * accel)
        return 2.0 * peak_speed / accel

    @staticmethod
    def _trapezoid_profile(distance: float, target_speed: float, accel: float) -> dict:
        if distance <= 1e-9:
            return {
                "distance": 0.0,
                "accel": 0.0,
                "cruise_speed": 0.0,
                "t_accel": 0.0,
                "t_cruise": 0.0,
                "t_total": 0.0,
                "triangular": False,
            }

        target_speed = max(1e-6, target_speed)
        accel = max(1e-6, accel)
        accel_dist = (target_speed * target_speed) / (2.0 * accel)

        if 2.0 * accel_dist <= distance:
            t_accel = target_speed / accel
            t_cruise = (distance - 2.0 * accel_dist) / target_speed
            cruise_speed = target_speed
            triangular = False
        else:
            cruise_speed = sqrt(distance * accel)
            t_accel = cruise_speed / accel
            t_cruise = 0.0
            triangular = True

        return {
            "distance": float(distance),
            "accel": float(accel),
            "cruise_speed": float(cruise_speed),
            "t_accel": float(t_accel),
            "t_cruise": float(t_cruise),
            "t_total": float((2.0 * t_accel) + t_cruise),
            "triangular": triangular,
        }

    def _linear_profile(
        self,
        dx: float,
        dy: float,
        dz: float,
        commanded_speed_mm_s: float,
        speed_limits: Dict[str, float] | None = None,
        accel_limits: Dict[str, float] | None = None,
    ) -> dict:
        distance = sqrt(dx * dx + dy * dy + dz * dz)
        if distance <= 1e-9:
            return self._trapezoid_profile(0.0, 0.0, 0.0)

        accel_limits = accel_limits or self.acceleration

        axis_accel_limits: List[float] = []
        for axis, delta in (("X", dx), ("Y", dy), ("Z", dz)):
            comp = abs(delta)
            if comp <= 1e-12:
                continue

            axis_accel_limits.append(accel_limits[axis] * (distance / comp))

        path_accel = min(axis_accel_limits) if axis_accel_limits else min(accel_limits["X"], accel_limits["Y"], accel_limits["Z"])

        target_speed = commanded_speed_mm_s
        return self._trapezoid_profile(distance, target_speed, path_accel)

    def _rotary_profile(
        self,
        da: float,
        commanded_speed_deg_s: float,
        max_speed: float | None = None,
        accel: float | None = None,
    ) -> dict:
        distance = abs(da)
        if distance <= 1e-9:
            return self._trapezoid_profile(0.0, 0.0, 0.0)

        target_speed = commanded_speed_deg_s
        target_accel = accel if accel is not None else self.acceleration["A"]
        return self._trapezoid_profile(distance, target_speed, target_accel)

    def move_linear(self, params: Dict[str, float]) -> MoveResult:
        if not self.motors_enabled:
            raise RuntimeError("Motors are disabled (M17 to enable)")

        if "F" in params:
            self.feedrate_mm_min = max(1.0, params["F"])

        start = dict(self.position)
        target = dict(self.position)

        for axis in ("X", "Y", "Z", "A"):
            if axis not in params:
                continue
            if self.absolute_mode:
                target[axis] = params[axis]
            else:
                target[axis] = self.position[axis] + params[axis]

        if self.soft_endstops:
            target["X"] = min(max(target["X"], self.limits.x_min), self.limits.x_max)
            target["Y"] = min(max(target["Y"], self.limits.y_min), self.limits.y_max)
            target["Z"] = min(max(target["Z"], self.limits.z_min), self.limits.z_max)

        dx = target["X"] - start["X"]
        dy = target["Y"] - start["Y"]
        dz = target["Z"] - start["Z"]
        da = target["A"] - start["A"]

        linear_speed_mm_s = self.feedrate_mm_min / 60.0
        linear_profile = self._linear_profile(dx, dy, dz, linear_speed_mm_s)
        linear_time = linear_profile["t_total"]

        rotary_command_speed = self.rotary_speed_deg_s
        if abs(dx) <= 1e-9 and abs(dy) <= 1e-9 and abs(dz) <= 1e-9:
            # For pure rotary moves, allow F to influence commanded A speed.
            rotary_command_speed = max(rotary_command_speed, linear_speed_mm_s)

        rotary_profile = self._rotary_profile(da, rotary_command_speed)
        rotary_time = rotary_profile["t_total"]

        duration_s = max(linear_time, rotary_time)

        self.position.update(target)
        self.path.append((self.position["X"], self.position["Y"], self.position["Z"]))

        motion = {
            "kind": "coordinated",
            "duration_s": float(duration_s),
            "start": {k: float(v) for k, v in start.items()},
            "end": {k: float(v) for k, v in target.items()},
            "linear": {
                "dx": float(dx),
                "dy": float(dy),
                "dz": float(dz),
                **linear_profile,
            },
            "rotary": {
                "da": float(da),
                **rotary_profile,
            },
        }

        return MoveResult(start=start, end=target, duration_s=duration_s, motion=motion)

    def set_position(self, params: Dict[str, float]) -> None:
        for axis in ("X", "Y", "Z", "A"):
            if axis in params:
                self.position[axis] = params[axis]

    def set_max_feedrate(self, params: Dict[str, float]) -> None:
        for axis in ("X", "Y", "Z", "A"):
            if axis in params:
                self.max_feedrate_mm_s[axis] = max(0.1, params[axis])

    def _home_segment_motion(
        self,
        start: Dict[str, float],
        end: Dict[str, float],
        axis: str,
        phase: str,
        commanded_speed: float,
    ) -> dict:
        dx = end["X"] - start["X"]
        dy = end["Y"] - start["Y"]
        dz = end["Z"] - start["Z"]
        da = end["A"] - start["A"]

        if axis == "A":
            linear_profile = self._trapezoid_profile(0.0, 0.0, 0.0)
            rotary_profile = self._rotary_profile(
                da,
                commanded_speed,
                max_speed=commanded_speed,
                accel=self.acceleration["A"],
            )
        else:
            speed_limits = {"X": 1e-6, "Y": 1e-6, "Z": 1e-6}
            speed_limits[axis] = max(0.1, commanded_speed)
            linear_profile = self._linear_profile(dx, dy, dz, max(0.1, commanded_speed), speed_limits=speed_limits)
            rotary_profile = self._trapezoid_profile(0.0, 0.0, 0.0)

        duration_s = max(linear_profile["t_total"], rotary_profile["t_total"])
        return {
            "kind": "home-segment",
            "axis": axis,
            "phase": phase,
            "duration_s": float(duration_s),
            "start": {k: float(v) for k, v in start.items()},
            "end": {k: float(v) for k, v in end.items()},
            "linear": {
                "dx": float(dx),
                "dy": float(dy),
                "dz": float(dz),
                **linear_profile,
            },
            "rotary": {
                "da": float(da),
                **rotary_profile,
            },
        }

    def home_sequence(self, axes: set[str]) -> HomeSequenceResult:
        if not self.motors_enabled:
            raise RuntimeError("Motors are disabled (M17 to enable)")

        if not axes:
            axes = {"X", "Y", "Z", "A"}

        ordered_axes = [axis for axis in ("X", "Y", "Z", "A") if axis in axes]
        start = dict(self.position)
        cursor = dict(self.position)

        backoff = {"X": 4.0, "Y": 4.0, "Z": 2.0, "A": 8.0}
        motions: List[dict] = []
        duration_s = 0.0

        for axis in ordered_axes:
            # Home position is fixed machine zero for all axes.
            home_pos = 0.0

            # 1) Fast seek toward endstop.
            seek_target = dict(cursor)
            seek_target[axis] = home_pos
            seek_motion = self._home_segment_motion(cursor, seek_target, axis, "seek-fast", self.homing_feedrate[axis])
            if seek_motion["duration_s"] > 0:
                motions.append(seek_motion)
                duration_s += seek_motion["duration_s"]
                cursor = seek_target
                self.path.append((cursor["X"], cursor["Y"], cursor["Z"]))

            # 2) Retract off switch.
            retract_pos = home_pos + backoff[axis]
            if axis == "X":
                retract_pos = min(retract_pos, self.limits.x_max)
            elif axis == "Y":
                retract_pos = min(retract_pos, self.limits.y_max)
            elif axis == "Z":
                retract_pos = min(retract_pos, self.limits.z_max)

            retract_target = dict(cursor)
            retract_target[axis] = retract_pos
            retract_motion = self._home_segment_motion(
                cursor,
                retract_target,
                axis,
                "retract",
                max(0.5, self.homing_feedrate[axis] * 0.5),
            )
            if retract_motion["duration_s"] > 0:
                motions.append(retract_motion)
                duration_s += retract_motion["duration_s"]
                cursor = retract_target
                self.path.append((cursor["X"], cursor["Y"], cursor["Z"]))

            # 3) Slow seek for accurate trigger point.
            slow_target = dict(cursor)
            slow_target[axis] = home_pos
            slow_motion = self._home_segment_motion(
                cursor,
                slow_target,
                axis,
                "seek-slow",
                max(0.5, self.homing_feedrate[axis] * 0.25),
            )
            if slow_motion["duration_s"] > 0:
                motions.append(slow_motion)
                duration_s += slow_motion["duration_s"]
                cursor = slow_target
                self.path.append((cursor["X"], cursor["Y"], cursor["Z"]))

        self.position.update(cursor)
        end = dict(self.position)
        return HomeSequenceResult(start=start, end=end, duration_s=float(duration_s), motions=motions)

    def home(self, axes: set[str]) -> MoveResult:
        seq = self.home_sequence(axes)
        motion = seq.motions[-1] if seq.motions else {
            "kind": "home",
            "duration_s": 0.0,
            "start": {k: float(v) for k, v in seq.start.items()},
            "end": {k: float(v) for k, v in seq.end.items()},
            "linear": {"dx": 0.0, "dy": 0.0, "dz": 0.0, **self._trapezoid_profile(0.0, 0.0, 0.0)},
            "rotary": {"da": 0.0, **self._trapezoid_profile(0.0, 0.0, 0.0)},
        }
        return MoveResult(start=seq.start, end=seq.end, duration_s=seq.duration_s, motion=motion)
