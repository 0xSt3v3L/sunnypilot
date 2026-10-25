#!/usr/bin/env python3
"""EPS probe: step the Toyota LKA torque command from 1500 to 3000 and record how the EPS answers. Closed areas only.

In the car (the drive to the venue runs normally: probe mode is off, the limit is 1500):
  1. Car off, device offroad: ssh in, echo -n 1 > /data/params/d/JoystickDebugMode
     Never change JoystickDebugMode while onroad. It clears itself when the car turns off.
  2. Car on (READY), ACC main on, D, foot on the brake, wheels straight. The screen shows the joystick alert and MADS.
  3. Stage 1, standstill:   cd /data/openpilot && python3 openpilot/tools/op_mads/eps_probe.py --stage 1
     Hands near (not on) the wheel, foot on the brake. Ctrl-C stops at once.
  4. Stage 2, creeping at 1-3 km/h on the brake, clear flat area, only if stage 1 printed ACCEPT for every level,
     "no lateral at this speed", NO-RESPONSE, or FAULT at 1500 that cleared after turning the car off and on:
       python3 openpilot/tools/op_mads/eps_probe.py --stage 2
     Never after a stage 1 FAULT above 1500.
  5. Car off. Drive home normally. Logs: the route's rlogs and /data/eps_probe/.

Per level and direction it prints ACCEPT (command reached the target, EPS >= 1.2 x command), CLIP, FAULT or
NO-RESPONSE (EPS < 100 for the whole hold).
"""
import argparse
import json
import statistics
import sys
import time
from dataclasses import dataclass
from pathlib import Path

DEFAULT_LEVELS = [1500, 1800, 2100, 2400, 2700, 3000]
HOLD_S = 1.5
REST_S = 2.5
RAMP_PER_S = 1500.  # the car controller ramps 15 per 10 ms frame
SETTLE_S = 0.5  # after the nominal ramp, before the hold: message latency and any ramp held back by the EPS error limit
DT = 0.01
STAGE_MAX_SPEED = {1: 0.3, 2: 5. / 3.6}  # m/s
MAX_STEER_RATE = 100.  # deg/s
MAX_STEER_ANGLE = 450.  # deg
MAX_LAT_LOST_S = 0.5
LAT_WAIT_S = 3.
ACCEPT_RATIO = 1.2
TARGET_TOLERANCE = 100.
NO_RESPONSE_EPS = 100.
LOG_DIR = Path("/data/eps_probe")


@dataclass
class Step:
  level: int
  target: int  # CAN torque units, positive = left
  direction: str  # "left", "right" or "rest"
  duration: float  # s
  hold_s: float  # s at the end of the step that count for the verdict


def build_schedule(levels: list[int], hold_s: float = HOLD_S, rest_s: float = REST_S) -> list[Step]:
  steps = []
  for level in levels:
    for sign, direction in ((1, "left"), (-1, "right")):
      steps.append(Step(level, sign * level, direction, level / RAMP_PER_S + SETTLE_S + hold_s, hold_s))
      steps.append(Step(level, 0, "rest", rest_s, 0.))
  return steps


def levels_from_args(levels: list[int] | None, max_level: int | None) -> list[int]:
  out = levels if levels else DEFAULT_LEVELS
  return [lv for lv in out if max_level is None or lv <= max_level]


def abort_reason(cs, lat_active: bool, stage: int, lat_lost_s: float) -> str | None:
  if cs.steerFaultTemporary or cs.steerFaultPermanent:
    return f"steer fault (temporary={cs.steerFaultTemporary}, permanent={cs.steerFaultPermanent})"
  if cs.steeringPressed:
    return "driver steering"
  if abs(cs.steeringRateDeg) > MAX_STEER_RATE:
    return f"steering rate {cs.steeringRateDeg:.0f} deg/s"
  if abs(cs.steeringAngleDeg) > MAX_STEER_ANGLE:
    return f"steering angle {cs.steeringAngleDeg:.0f} deg"
  if cs.vEgo > STAGE_MAX_SPEED[stage]:
    return f"speed {cs.vEgo * 3.6:.1f} km/h above the stage {stage} limit"
  if lat_lost_s > MAX_LAT_LOST_S:
    return "lateral lost (MADS paused or disengaged)"
  return None


def level_verdict(target: int, cmds: list[float], eps: list[float], faulted: bool) -> str:
  if faulted:
    return "FAULT"
  if not eps or max(abs(e) for e in eps) < NO_RESPONSE_EPS:
    return "NO-RESPONSE"
  sign = 1 if target >= 0 else -1
  cmd = statistics.median(abs(c) for c in cmds)
  ratio = statistics.median(e * sign / abs(c) for c, e in zip(cmds, eps, strict=True) if c != 0) if cmd > 0 else 0.
  return "ACCEPT" if cmd >= abs(target) - TARGET_TOLERANCE and ratio >= ACCEPT_RATIO else "CLIP"


def probe_ready(CP) -> bool:
  from opendbc.car.toyota.eps_probe import eps_probe_active
  return eps_probe_active(CP)


class _Log:
  def __init__(self, path: Path):
    path.parent.mkdir(parents=True, exist_ok=True)
    self.f = path.open("a")
    print(f"log: {path}")

  def write(self, event: str, **kw) -> None:
    self.f.write(json.dumps({"event": event, "mono_ns": time.monotonic_ns(), **kw}) + "\n")
    self.f.flush()


def run(stage: int, levels: list[int]) -> int:
  from openpilot.cereal import messaging
  from opendbc.car.structs import car
  from opendbc.car.toyota.eps_probe import EPS_PROBE_STEER_MAX
  from openpilot.common.params import Params
  from openpilot.common.realtime import Ratekeeper

  params = Params()
  cp_bytes = params.get("CarParams")
  CP = messaging.log_from_bytes(cp_bytes, car.CarParams) if cp_bytes else None
  if CP is None or not probe_ready(CP):
    print("probe mode is not active: set JoystickDebugMode while offroad, then turn the car on")
    return 2

  route = params.get("CurrentRoute") or "noroute"
  log = _Log(LOG_DIR / f"{route}--{time.strftime('%Y%m%d-%H%M%S')}.jsonl")
  sm = messaging.SubMaster(['carState', 'carControl', 'carOutput'])
  pm = messaging.PubMaster(['testJoystick'])
  rk = Ratekeeper(int(1 / DT), print_delay_threshold=None)

  def publish(target: float) -> None:
    msg = messaging.new_message('testJoystick')
    msg.testJoystick.axes = [0., target / EPS_PROBE_STEER_MAX]
    msg.testJoystick.buttons = [False]
    pm.send('testJoystick', msg)

  steps = build_schedule(levels)
  log.write("start", stage=stage, levels=levels, steer_max=EPS_PROBE_STEER_MAX)
  try:
    # joystickd needs testJoystick to keep lateral; wait for MADS lateral to come up
    waited = 0.
    while True:
      publish(0.)
      sm.update(0)
      if sm['carControl'].latActive:
        break
      waited += DT
      if waited > LAT_WAIT_S:
        print("no lateral at this speed (latActive never came up)")
        log.write("abort", reason="no lateral at this speed")
        return 1
      rk.keep_time()

    lat_lost_s = 0.
    applies = [s for s in steps if s.direction != "rest"]
    for step in steps:
      cmds, eps, faulted = [], [], False
      frames = int(round(step.duration / DT))
      hold_from = frames - int(round(step.hold_s / DT))
      log.write("step", level=step.level, target=step.target, direction=step.direction, duration=step.duration)
      for i in range(frames):
        publish(step.target)
        sm.update(0)
        cs = sm['carState']
        lat_lost_s = 0. if sm['carControl'].latActive else lat_lost_s + DT
        reason = abort_reason(cs, sm['carControl'].latActive, stage, lat_lost_s)
        if step.direction != "rest" and i >= hold_from:
          cmds.append(float(sm['carOutput'].actuatorsOutput.torqueOutputCan))
          eps.append(float(cs.steeringTorqueEps))
        if reason is not None:
          faulted = cs.steerFaultTemporary or cs.steerFaultPermanent
          if step.direction != "rest" and faulted:
            print(f"{step.direction:5s} {step.level}: FAULT")
            log.write("verdict", level=step.level, target=step.target, verdict="FAULT")
          print(f"ABORT: {reason} (level {step.level} {step.direction})")
          log.write("abort", reason=reason, level=step.level, target=step.target)
          return 1
        rk.keep_time()
      if step.direction != "rest":
        verdict = level_verdict(step.target, cmds, eps, faulted)
        cmd = statistics.median(abs(c) for c in cmds) if cmds else 0.
        eps_med = statistics.median(abs(e) for e in eps) if eps else 0.
        n = applies.index(step) + 1
        print(f"{n:2d}/{len(applies)} {step.direction:5s} {step.level}: {verdict:11s} cmd {cmd:5.0f}  eps {eps_med:5.0f}")
        log.write("verdict", level=step.level, target=step.target, verdict=verdict, cmd=cmd, eps=eps_med)
    log.write("end")
    print("done")
    return 0
  except KeyboardInterrupt:
    print("stopped")
    log.write("abort", reason="ctrl-c")
    return 1
  finally:
    for _ in range(30):
      publish(0.)
      time.sleep(DT)


def main() -> int:
  parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
  parser.add_argument("--stage", type=int, choices=(1, 2), required=True, help="1: standstill, 2: creeping at 1-3 km/h")
  parser.add_argument("--levels", type=int, nargs="+", help="torque levels (default: %(default)s)")
  parser.add_argument("--max", type=int, dest="max_level", help="stop at this level")
  parser.add_argument("--dry-run", action="store_true", help="print the schedule and exit")
  args = parser.parse_args()

  levels = levels_from_args(args.levels, args.max_level)
  if args.dry_run:
    steps = build_schedule(levels)
    for s in steps:
      print(f"{s.direction:5s} target {s.target:6d}  {s.duration:4.2f} s")
    print(f"total {sum(s.duration for s in steps):.1f} s, stage {args.stage}")
    return 0
  return run(args.stage, levels)


if __name__ == "__main__":
  sys.exit(main())
