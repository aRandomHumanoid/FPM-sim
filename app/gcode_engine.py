from __future__ import annotations

import asyncio
from dataclasses import dataclass, field
from typing import Awaitable, Callable, Dict, List, Optional

import numpy as np

from .gcode_parser import GcodeParser, ParsedCommand
from .machine import PrinterState
from .probing import ProbeWorld


StatePublisher = Callable[[dict], Awaitable[None]]
MessagePublisher = Callable[[str], Awaitable[None]]
MotionPublisher = Callable[[dict], Awaitable[None]]


@dataclass(slots=True)
class ExecResult:
    messages: List[str] = field(default_factory=list)
    ok: bool = True
    duration_s: float = 0.0
    motion: dict | None = None
    motions: List[dict] = field(default_factory=list)


class MarlinEngine:
    def __init__(
        self,
        state: PrinterState,
        probe_world: ProbeWorld,
        publish_state: StatePublisher,
        publish_message: MessagePublisher,
        publish_motion: MotionPublisher | None = None,
    ) -> None:
        self.state = state
        self.probe_world = probe_world
        self.publish_state = publish_state
        self.publish_message = publish_message
        self.publish_motion = publish_motion
        self.parser = GcodeParser()
        self.sd_printing = False
        self.prompt_active = False
        self.last_prompt_response: int | None = None
        self.abort_wait_requested = False

    def _consume_abort_requested(self) -> bool:
        if self.abort_wait_requested:
            self.abort_wait_requested = False
            return True
        return False

    async def _sleep_with_interrupt(self, duration_s: float) -> bool:
        elapsed = 0.0
        step = 0.05
        while elapsed < duration_s:
            if self._consume_abort_requested():
                return True
            dt = min(step, duration_s - elapsed)
            await asyncio.sleep(dt)
            elapsed += dt
        return False

    async def execute_line(self, line: str) -> None:
        maybe = self.parser.parse(line)
        if maybe is None:
            return

        try:
            result = await self._execute(maybe)
        except Exception as exc:
            await self.publish_message(f"Error:{exc}")
            return

        if result.motions:
            for motion in result.motions:
                if self.publish_motion is not None:
                    await self.publish_motion(motion)

                seg_duration = float(motion.get("duration_s", 0.0))
                if seg_duration > 0.75:
                    await self._busy_wait(seg_duration)
                elif seg_duration > 0:
                    interrupted = await self._sleep_with_interrupt(seg_duration)
                    if interrupted:
                        await self.publish_message("echo:busy: interrupted")
        else:
            if result.motion is not None and self.publish_motion is not None:
                await self.publish_motion(result.motion)

            if result.duration_s > 0.75:
                await self._busy_wait(result.duration_s)
            elif result.duration_s > 0:
                interrupted = await self._sleep_with_interrupt(result.duration_s)
                if interrupted:
                    await self.publish_message("echo:busy: interrupted")

        for msg in result.messages:
            await self.publish_message(msg)

        if result.ok:
            await self.publish_message("ok")

        await self.publish_state(self.state.snapshot())

    async def _busy_wait(self, duration_s: float) -> None:
        elapsed = 0.0
        step = 1.0
        while elapsed + step < duration_s:
            if self._consume_abort_requested():
                await self.publish_message("echo:busy: interrupted")
                return
            await asyncio.sleep(step)
            elapsed += step
            await self.publish_message("echo:busy: processing")

        remaining = duration_s - elapsed
        if remaining > 0:
            interrupted = await self._sleep_with_interrupt(remaining)
            if interrupted:
                await self.publish_message("echo:busy: interrupted")

    async def _execute(self, cmd: ParsedCommand) -> ExecResult:
        if cmd.letter == "G":
            return self._exec_g(cmd)
        return self._exec_m(cmd)

    def _probe_motion(self, cmd: ParsedCommand, start: np.ndarray, end: np.ndarray) -> tuple[float, dict]:
        if "F" in cmd.params:
            self.state.feedrate_mm_min = max(1.0, cmd.params["F"])

        dx = float(end[0] - start[0])
        dy = float(end[1] - start[1])
        dz = float(end[2] - start[2])
        linear_speed_mm_s = self.state.feedrate_mm_min / 60.0
        linear_profile = self.state._linear_profile(dx, dy, dz, linear_speed_mm_s)
        duration_s = float(linear_profile["t_total"])

        a = float(self.state.position["A"])
        motion = {
            "kind": "coordinated",
            "duration_s": duration_s,
            "start": {
                "X": float(start[0]),
                "Y": float(start[1]),
                "Z": float(start[2]),
                "A": a,
            },
            "end": {
                "X": float(end[0]),
                "Y": float(end[1]),
                "Z": float(end[2]),
                "A": a,
            },
            "linear": {
                "dx": dx,
                "dy": dy,
                "dz": dz,
                **linear_profile,
            },
            "rotary": {
                "da": 0.0,
                "distance": 0.0,
                "accel": 0.0,
                "cruise_speed": 0.0,
                "t_accel": 0.0,
                "t_cruise": 0.0,
                "t_total": 0.0,
                "triangular": False,
            },
            "command": cmd.normalized,
        }

        return duration_s, motion

    def _exec_g(self, cmd: ParsedCommand) -> ExecResult:
        if cmd.code in (0.0, 1.0):
            move = self.state.move_linear(cmd.params)
            motion = dict(move.motion or {})
            motion["command"] = cmd.normalized
            return ExecResult(duration_s=move.duration_s, motion=motion)

        if cmd.code == 28.0:
            axes = {a for a in ("X", "Y", "Z", "A") if a in cmd.params}
            seq = self.state.home_sequence(axes)
            motions = []
            for idx, motion in enumerate(seq.motions):
                payload = dict(motion)
                payload["command"] = cmd.normalized
                payload["segment_index"] = idx
                payload["segment_count"] = len(seq.motions)
                motions.append(payload)
            return ExecResult(messages=["echo:Homing done"], duration_s=seq.duration_s, motions=motions)

        if cmd.code == 30.0:
            x = self.state.position["X"]
            y = self.state.position["Y"]
            z = self.state.position["Z"]
            trigger_z = 0.0

            # G30 is modeled as probing against a fixed bed plane at Z=0.
            if z > trigger_z:
                hit_z = trigger_z
                travel = z - trigger_z
            else:
                # Already at or below the trigger plane, so trigger immediately.
                hit_z = z
                travel = 0.0

            self.state.position["Z"] = float(hit_z)
            self.state.path.append((self.state.position["X"], self.state.position["Y"], self.state.position["Z"]))
            msg = f"Bed X:{x:.3f} Y:{y:.3f} Z:{hit_z:.4f}"
            # Probe commands complete immediately to avoid host-side timeout behavior.
            return ExecResult(messages=[msg], duration_s=0.0)

        if cmd.code == 4.0:
            dwell_s = 0.0
            if "P" in cmd.params:
                dwell_s = max(0.0, cmd.params["P"] / 1000.0)
            if "S" in cmd.params:
                dwell_s = max(dwell_s, cmd.params["S"])
            return ExecResult(duration_s=dwell_s)

        if cmd.code == 90.0:
            self.state.absolute_mode = True
            return ExecResult(messages=["echo:Absolute positioning"])

        if cmd.code == 91.0:
            self.state.absolute_mode = False
            return ExecResult(messages=["echo:Relative positioning"])

        if cmd.code == 92.0:
            self.state.set_position(cmd.params)
            self.state.path.append((self.state.position["X"], self.state.position["Y"], self.state.position["Z"]))
            return ExecResult()

        if cmd.code in (38.2, 38.3, 38.4, 38.5):
            return self._exec_probe_cycle(cmd)

        return ExecResult(messages=[f"Error:Unsupported G-code G{cmd.code}"], ok=False)

    def _exec_probe_cycle(self, cmd: ParsedCommand) -> ExecResult:
        start = np.array(
            [self.state.position["X"], self.state.position["Y"], self.state.position["Z"]],
            dtype=np.float64,
        )

        end = np.array(start, copy=True)
        for i, axis in enumerate(("X", "Y", "Z")):
            if axis in cmd.params:
                if self.state.absolute_mode:
                    end[i] = cmd.params[axis]
                else:
                    end[i] = start[i] + cmd.params[axis]

        if cmd.code in (38.2, 38.3):
            # Probe toward target and stop on contact.
            hit = self.probe_world.probe_segment(start, end)
            must_hit = cmd.code == 38.2

            if hit is not None:
                final = np.array(hit.point, dtype=np.float64)
            else:
                final = end

            duration_s, motion = self._probe_motion(cmd, start, final)
            self.state.position["X"] = float(final[0])
            self.state.position["Y"] = float(final[1])
            self.state.position["Z"] = float(final[2])
            self.state.path.append((self.state.position["X"], self.state.position["Y"], self.state.position["Z"]))

            if hit is not None:
                return ExecResult(
                    messages=[
                        f"echo:probe hit {hit.mesh_name} X:{hit.point[0]:.4f} Y:{hit.point[1]:.4f} Z:{hit.point[2]:.4f}"
                    ],
                    duration_s=duration_s,
                    motion=motion,
                )

            if must_hit:
                return ExecResult(messages=["Error:Probe target not reached"], ok=False, duration_s=duration_s, motion=motion)

            # G38.3: miss is allowed; move to target.
            return ExecResult(messages=["echo:probe miss"], duration_s=duration_s, motion=motion)

        # G38.4 / G38.5: probe away from target and stop on contact break.
        break_hit = self.probe_world.probe_contact_break_segment(start, end)
        must_break = cmd.code == 38.4

        if break_hit is not None:
            final = np.array(break_hit.point, dtype=np.float64)
        else:
            final = end

        duration_s, motion = self._probe_motion(cmd, start, final)
        self.state.position["X"] = float(final[0])
        self.state.position["Y"] = float(final[1])
        self.state.position["Z"] = float(final[2])
        self.state.path.append((self.state.position["X"], self.state.position["Y"], self.state.position["Z"]))

        if break_hit is not None:
            return ExecResult(
                messages=[
                    f"echo:probe break X:{break_hit.point[0]:.4f} Y:{break_hit.point[1]:.4f} Z:{break_hit.point[2]:.4f}"
                ],
                duration_s=duration_s,
                motion=motion,
            )

        if must_break:
            return ExecResult(messages=["Error:Probe contact break not reached"], ok=False, duration_s=duration_s, motion=motion)

        # G38.5: break miss is allowed; move to target.
        return ExecResult(messages=["echo:probe break miss"], duration_s=duration_s, motion=motion)

    def _exec_m(self, cmd: ParsedCommand) -> ExecResult:
        mcode = int(cmd.code)

        if mcode == 17:
            self.state.motors_enabled = True
            return ExecResult(messages=["echo:Motors enabled"])

        if mcode in (18, 84):
            self.state.motors_enabled = False
            return ExecResult(messages=["echo:Motors disabled"])

        if mcode == 114:
            p = self.state.position
            pos = f"X:{p['X']:.3f} Y:{p['Y']:.3f} Z:{p['Z']:.3f} A:{p['A']:.3f}"
            count = f"Count X:{int(p['X']*80)} Y:{int(p['Y']*80)} Z:{int(p['Z']*400)} A:{int(p['A']*10)}"
            return ExecResult(messages=[f"{pos} {count}"])

        if mcode == 210:
            if "A" in cmd.params:
                self.state.rotary_speed_deg_s = max(0.1, cmd.params["A"])
                return ExecResult(messages=[f"echo:A max speed set to {self.state.rotary_speed_deg_s:.2f} deg/s"])
            return ExecResult(messages=[f"echo:A max speed {self.state.rotary_speed_deg_s:.2f} deg/s"])

        if mcode == 211:
            if "S" in cmd.params:
                self.state.soft_endstops = bool(int(cmd.params["S"]))
            mode = "on" if self.state.soft_endstops else "off"
            return ExecResult(messages=[f"echo:Soft endstops {mode}"])

        if mcode == 201:
            for axis in ("X", "Y", "Z", "A"):
                if axis in cmd.params:
                    self.state.acceleration[axis] = max(1.0, cmd.params[axis])
            accel = " ".join(f"{k}{v:.1f}" for k, v in self.state.acceleration.items())
            return ExecResult(messages=[f"echo:Acceleration {accel}"])

        if mcode == 203:
            if any(axis in cmd.params for axis in ("X", "Y", "Z", "A")):
                self.state.set_max_feedrate(cmd.params)
            fr = " ".join(f"{k}{v:.1f}" for k, v in self.state.max_feedrate_mm_s.items())
            return ExecResult(messages=[f"echo:Max feedrate {fr} (mm/s)"])

        if mcode == 204:
            if "S" in cmd.params:
                val = max(1.0, cmd.params["S"])
                for axis in ("X", "Y", "Z"):
                    self.state.acceleration[axis] = val
            if "A" in cmd.params:
                self.state.acceleration["A"] = max(1.0, cmd.params["A"])
            accel = " ".join(f"{k}{v:.1f}" for k, v in self.state.acceleration.items())
            return ExecResult(messages=[f"echo:Accel profile {accel}"])

        if mcode == 503:
            s = 1 if self.state.soft_endstops else 0
            lines = [
                "echo:Settings Stored (simulated):",
                (
                    "echo:  M201 "
                    f"X{self.state.acceleration['X']:.1f} "
                    f"Y{self.state.acceleration['Y']:.1f} "
                    f"Z{self.state.acceleration['Z']:.1f} "
                    f"A{self.state.acceleration['A']:.1f}"
                ),
                (
                    "echo:  M203 "
                    f"X{self.state.max_feedrate_mm_s['X']:.1f} "
                    f"Y{self.state.max_feedrate_mm_s['Y']:.1f} "
                    f"Z{self.state.max_feedrate_mm_s['Z']:.1f} "
                    f"A{self.state.max_feedrate_mm_s['A']:.1f}"
                ),
                (
                    "echo:  M204 "
                    f"S{self.state.acceleration['X']:.1f} "
                    f"A{self.state.acceleration['A']:.1f}"
                ),
                f"echo:  M210 A{self.state.rotary_speed_deg_s:.2f}",
                f"echo:  M211 S{s}",
            ]
            return ExecResult(messages=lines)

        if mcode in (0, 1):
            self.prompt_active = True
            self.sd_printing = False
            return ExecResult(
                messages=[
                    "echo:action:prompt_begin Continue print?",
                    "echo:action:prompt_button Continue",
                    "echo:action:prompt_button Cancel",
                    "echo:action:prompt_end",
                    "echo:awaiting M876 S<choice>",
                ]
            )

        if mcode == 24:
            self.sd_printing = True
            self.prompt_active = False
            return ExecResult(messages=["echo:SD print resumed"])

        if mcode == 25:
            self.sd_printing = False
            return ExecResult(messages=["echo:SD print paused"])

        if mcode == 108:
            self.abort_wait_requested = True
            self.prompt_active = False
            self.sd_printing = True
            return ExecResult(messages=["echo:Wait canceled"])

        if mcode == 876:
            if "S" in cmd.params:
                choice = int(cmd.params["S"])
                self.last_prompt_response = choice
                self.prompt_active = False
                self.sd_printing = True
                return ExecResult(messages=[f"echo:Prompt response {choice}"])

            active = "yes" if self.prompt_active else "no"
            resp = "none" if self.last_prompt_response is None else str(self.last_prompt_response)
            return ExecResult(messages=[f"echo:Prompt active={active} last_response={resp}"])

        # Common host keepalive poll command; useful for UIs/scripts.
        if mcode == 105:
            return ExecResult(messages=["T:25.0 /0.0 B:25.0 /0.0"])

        if mcode == 110:
            return ExecResult(messages=["echo:Line number reset"])

        if mcode == 115:
            return ExecResult(messages=["FIRMWARE_NAME:FPM-Sim PROTOCOL_VERSION:1.0 MACHINE_TYPE:4AxisSim"])

        if mcode == 400:
            return ExecResult(messages=["echo:Planner synchronized"])

        return ExecResult(messages=[f"Error:Unsupported M-code M{mcode}"], ok=False)
