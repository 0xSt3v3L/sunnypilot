#!/usr/bin/env python3
"""EPS probe: step the Toyota LKA torque command and record how the EPS answers. Closed areas only.

Two modes. Level mode (default) steps to 1500 and back; the EPS faults above 1500 (route 000000d1). Rate sweep (--sweep)
ramps 0 -> 1500 -> 0 left and right and then +1500 -> -1500 at each rate under test (per 10 ms frame), to find the
fastest up/down rate the EPS takes without a fault; it stops at the first fault. Probe mode allows up to 90 per frame.

In the car (the drive to the venue runs normally: probe mode is off, the limit is 1500):
  1. Car off, device offroad: ssh in, echo -n 1 > /data/params/d/JoystickDebugMode
     Never change JoystickDebugMode while onroad. It clears itself when the car turns off.
  2. Car on (READY), ACC main on, D, foot on the brake, wheels straight. The screen shows the joystick alert and MADS.
  3. Stage 1, standstill, inside tmux so a dropped SSH session does not matter (tmux new -s probe; reattach with
     tmux attach -t probe):   cd /data/openpilot && python3 openpilot/tools/op_mads/eps_probe.py --stage 1
     Hands near (not on) the wheel, foot on the brake.
     To stop at once: Ctrl-C, or press ACC main off (MADS drops lateral, the panda revokes it), or hold the wheel
     (driver steering aborts the run and the car controller cuts torque).
     Rate sweep instead: python3 openpilot/tools/op_mads/eps_probe.py --stage 1 --sweep   (about 70 s)
     It prints the rate the command reached and how fast the EPS followed for each step, and ends with the highest
     rate without a fault.
  4. Level mode: the last line says what to do next: "stage 2: GO ...", "stage 2: NO ..." or "stage 2: UNDECIDED ...".
     Stage 2, creeping at 1-3 km/h on the brake in a clear flat area, only on GO:
       python3 openpilot/tools/op_mads/eps_probe.py --stage 2
     A fault at 1500 needs the car off and on first, and JoystickDebugMode set again while offroad (it clears when the
     car turns off).
  5. Car off. Drive home normally. Logs: the route's rlogs and /data/eps_probe/.

Per level and direction it prints ACCEPT (command reached the target, EPS >= 1.2 x command), CLIP, FAULT or
NO-RESPONSE (EPS < 100 for the whole hold).
"""
import argparse
import fcntl
import json
import signal
import statistics
import sys
import time
from dataclasses import dataclass
from pathlib import Path

DEFAULT_LEVELS = [1500]
DEFAULT_RATES = [30, 45, 60, 75, 90]
RATE_MAX = 90  # per frame, the probe-mode limit of the car controller and the panda (EPS_PROBE_RATE_MAX)
LEVEL_RATE = 24  # level mode ramps like rate24 at low speed
SWEEP_LEVEL = 1500
SWEEP_SETTLE_S = 0.3
SWEEP_HOLD_S = 1.0
SWEEP_REST_S = 1.5
SWEEP_FINAL_REST_S = 2.0
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
REPO_ROOT = Path(__file__).resolve().parents[3]
NO_LATERAL = "no lateral at this speed"
PANDA_BLOCKED = "panda blocked steering: firmware without probe support, not an EPS result"


@dataclass
class Step:
  level: int
  target: int  # CAN torque units, positive = left
  direction: str  # "left", "right" or "rest"
  duration: float  # s
  hold_s: float  # s at the end of the step that count for the verdict
  rate: int = 0  # per frame the car controller ramps at (rate sweep, sent on joystick axis 2); 0 = LEVEL_RATE


def build_schedule(levels: list[int], hold_s: float = HOLD_S, rest_s: float = REST_S) -> list[Step]:
  steps = []
  for level in levels:
    for sign, direction in ((1, "left"), (-1, "right")):
      steps.append(Step(level, sign * level, direction, level / RAMP_PER_S + SETTLE_S + hold_s, hold_s))
      steps.append(Step(level, 0, "rest", rest_s, 0.))
  return steps


def build_sweep(rates: list[int]) -> list[Step]:
  """Per rate: left 0 -> 1500 -> 0, right 0 -> -1500 -> 0, then +1500 -> -1500 (reversal) -> 0."""
  steps = []
  ramp = SWEEP_LEVEL / 100.  # 1500 in frames-per-unit-rate: duration of a 0 <-> 1500 ramp is ramp / rate s
  for r in rates:
    steps += [
      Step(SWEEP_LEVEL, SWEEP_LEVEL, "left", ramp / r + SWEEP_SETTLE_S + SWEEP_HOLD_S, SWEEP_HOLD_S, r),
      Step(SWEEP_LEVEL, 0, "rest", ramp / r + SWEEP_REST_S, 0., r),
      Step(SWEEP_LEVEL, -SWEEP_LEVEL, "right", ramp / r + SWEEP_SETTLE_S + SWEEP_HOLD_S, SWEEP_HOLD_S, r),
      Step(SWEEP_LEVEL, 0, "rest", ramp / r + SWEEP_REST_S, 0., r),
      Step(SWEEP_LEVEL, SWEEP_LEVEL, "left", ramp / r + SWEEP_SETTLE_S + SWEEP_HOLD_S, SWEEP_HOLD_S, r),
      Step(SWEEP_LEVEL, -SWEEP_LEVEL, "reverse", 2 * ramp / r + SWEEP_SETTLE_S + SWEEP_HOLD_S, SWEEP_HOLD_S, r),
      Step(SWEEP_LEVEL, 0, "rest", ramp / r + SWEEP_FINAL_REST_S, 0., r),
    ]
  return steps


def joystick_axes(target: float, rate: int) -> list[float]:
  """testJoystick axes: 0 accel (unused), 1 torque as a fraction of 1500, 2 the ramp rate card applies in probe mode."""
  return [0., target / SWEEP_LEVEL, float(rate if rate > 0 else LEVEL_RATE)]


def transition_rate(cmds: list[float], start: float, end: float) -> float:
  """Units per frame the command covered between 10% and 90% of a start -> end change (100 Hz samples)."""
  span = end - start
  progress = [(c - start) / span for c in cmds]

  def crossing(level: float) -> float | None:
    i = next((i for i, p in enumerate(progress) if p >= level), None)
    if i is None or i == 0:
      return None if i is None else 0.
    return i - 1 + (level - progress[i - 1]) / (progress[i] - progress[i - 1])  # linear between samples

  t10, t90 = crossing(0.1), crossing(0.9)
  if t10 is None or t90 is None or t90 <= t10:
    return 0.
  return 0.8 * abs(span) / (t90 - t10)


def max_step(cmds: list[float]) -> float:
  """Largest change between consecutive samples. Message timing can make one frame catch up (up to the 90 limit), so
  the rlog's CAN commands are the exact record; this is the in-car approximation."""
  return max((abs(b - a) for a, b in zip(cmds, cmds[1:], strict=False)), default=0.)


def reach_time(values: list[float], threshold: float) -> float | None:
  """Seconds (100 Hz samples) until values first reach threshold, in its direction."""
  sign = 1 if threshold >= 0 else -1
  i = next((i for i, v in enumerate(values) if v * sign >= threshold * sign), None)
  return i * DT if i is not None else None


def rates_from_args(rates: list[int] | None) -> list[int]:
  out = rates if rates else DEFAULT_RATES
  for r in out:
    if not 0 < r <= RATE_MAX:
      raise ValueError(f"rate {r} outside 1..{RATE_MAX}")
  return out


def sweep_summary(results: list[tuple[int, str]], abort: str | None) -> str:
  passed = [r for r, v in results if v == "ACCEPT"]
  faulted = [r for r, v in results if v == "FAULT"]
  best = f"highest rate without a fault: {max(passed)}" if passed else ""
  if faulted:
    return f"{best} (faulted at {faulted[0]})" if passed else f"faulted at {faulted[0]}: no rate passed"
  if abort is not None:
    return f"stopped early ({abort}): {best or 'no rate finished'}"
  return best


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


def tx_blocked_reason(start: int, now: int) -> str | None:
  return PANDA_BLOCKED if now > start else None


def install_signal_handlers() -> None:
  """SIGHUP (SSH dropped) and SIGTERM stop the run like Ctrl-C, so the zero command still goes out."""
  def handler(signum, frame):
    raise KeyboardInterrupt
  for sig in (signal.SIGHUP, signal.SIGTERM):
    signal.signal(sig, handler)


def acquire_lock(path: Path):
  """An exclusive lock so a second instance cannot interleave its targets; None if another one holds it."""
  path.parent.mkdir(parents=True, exist_ok=True)
  f = path.open("w")
  try:
    fcntl.flock(f, fcntl.LOCK_EX | fcntl.LOCK_NB)
  except OSError:
    f.close()
    return None
  return f


def stage2_decision(stage: int, results: list[tuple[int, str]], abort: str | None) -> str:
  """The spec's stage 2 rule for a stage 1 run: results are (level, verdict) per apply step, abort the abort reason."""
  if stage != 1:
    return ""
  if abort == NO_LATERAL:
    return "stage 2: GO (no lateral at standstill)"
  if abort == PANDA_BLOCKED:
    return f"stage 2: UNDECIDED, stop: {PANDA_BLOCKED}; check the panda flash at home"
  faults = [lv for lv, v in results if v == "FAULT"]
  if faults:
    if min(faults) <= DEFAULT_LEVELS[0]:
      return ("stage 2: GO only if the fault clears: car off, set JoystickDebugMode again while offroad, car on, " +
              "no fault shown and MADS active")
    return f"stage 2: NO (the EPS faults at {min(faults)}); stop for the day"
  if abort is not None:
    return f"stage 2: UNDECIDED: fix the cause ({abort}) and rerun stage 1"
  verdicts = {v for _, v in results}
  if verdicts == {"ACCEPT"}:
    return "stage 2: GO (the EPS accepts every level at standstill)"
  if verdicts == {"NO-RESPONSE"}:
    return "stage 2: GO (the EPS ignores torque at standstill)"
  base = {v for lv, v in results if lv <= DEFAULT_LEVELS[0]}
  above = {v for lv, v in results if lv > DEFAULT_LEVELS[0]}
  if base == {"ACCEPT"} and "CLIP" in above and above <= {"ACCEPT", "CLIP"}:
    top = max(lv for lv, v in results if v == "ACCEPT")
    return f"stage 2: NO (result: the EPS clips above {top}); stop, analyze at home"
  return "stage 2: UNDECIDED, stop: mixed results, analyze at home"


def ensure_repo_on_path(path: list[str]) -> None:
  """An SSH shell on the device has no PYTHONPATH (manager gets /data/openpilot from launch_chffrplus.sh)."""
  if str(REPO_ROOT) not in path:
    path.insert(0, str(REPO_ROOT))


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


def run(stage: int, steps: list[Step], sweep: bool = False) -> int:
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
  lock = acquire_lock(LOG_DIR / ".lock")
  if lock is None:
    print("another eps_probe.py is running (reattach with: tmux attach -t probe)")
    return 2
  install_signal_handlers()

  route = params.get("CurrentRoute") or "noroute"
  log = _Log(LOG_DIR / f"{route}--{time.strftime('%Y%m%d-%H%M%S')}.jsonl")
  sm = messaging.SubMaster(['carState', 'carControl', 'carOutput', 'pandaStates'])
  pm = messaging.PubMaster(['testJoystick'])
  rk = Ratekeeper(int(1 / DT), print_delay_threshold=None)

  def publish(target: float, rate: int = 0) -> None:
    msg = messaging.new_message('testJoystick')
    msg.testJoystick.axes = joystick_axes(target, rate)
    msg.testJoystick.buttons = [False]
    pm.send('testJoystick', msg)

  def tx_blocked() -> int:
    return sum(ps.safetyTxBlocked for ps in sm['pandaStates'])

  results: list[tuple[int, str]] = []
  abort: str | None = None
  log.write("start", stage=stage, sweep=sweep, levels=sorted({s.level for s in steps}), rates=sorted({s.rate for s in steps}),
            steer_max=EPS_PROBE_STEER_MAX)
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
        abort = NO_LATERAL
        print(f"{NO_LATERAL} (latActive never came up)")
        log.write("abort", reason=NO_LATERAL)
        return 1
      rk.keep_time()

    blocked_start = tx_blocked()
    lat_lost_s = 0.
    n_applies = sum(s.direction != "rest" for s in steps)
    n = 0
    target_now = 0.  # the last target sent; the car controller ramps from it at step.rate
    for k, step in enumerate(steps):
      cmds, eps, all_cmds, all_eps = [], [], [], []
      start_target = target_now
      frames = int(round(step.duration / DT))
      hold_from = frames - int(round(step.hold_s / DT))
      log.write("step", level=step.level, target=step.target, direction=step.direction, duration=step.duration, rate=step.rate)
      for i in range(frames):
        target_now = float(step.target)
        publish(target_now, step.rate)
        sm.update(0)
        cs = sm['carState']
        lat_lost_s = 0. if sm['carControl'].latActive else lat_lost_s + DT
        reason = abort_reason(cs, sm['carControl'].latActive, stage, lat_lost_s) or tx_blocked_reason(blocked_start, tx_blocked())
        all_cmds.append(float(sm['carOutput'].actuatorsOutput.torqueOutputCan))
        all_eps.append(float(cs.steeringTorqueEps))
        if step.direction != "rest" and i >= hold_from:
          cmds.append(all_cmds[-1])
          eps.append(all_eps[-1])
        if reason is not None:
          abort = reason
          if cs.steerFaultTemporary or cs.steerFaultPermanent:
            results.append((step.rate if sweep else step.level, "FAULT"))
            print(f"{step.direction:5s} {step.level}: FAULT")
            log.write("verdict", level=step.level, target=step.target, verdict="FAULT")
          print(f"ABORT: {reason} (level {step.level} {step.direction})")
          log.write("abort", reason=reason, level=step.level, target=step.target)
          return 1
        rk.keep_time()
      if sweep:
        # how fast the command moved and how soon the EPS followed (2000 is ~1.4 x 1500; release: under 300)
        achieved = transition_rate(all_cmds, start_target, step.target) if step.target != start_target else 0.
        eps_t = reach_time(all_eps, 2000. if step.target > 0 else -2000. if step.target < 0 else 0.)
        if step.target == 0:
          eps_t = next((i * DT for i, e in enumerate(all_eps) if abs(e) < 300.), None)
        verdict = level_verdict(step.target, cmds, eps, False) if step.direction != "rest" else "-"
        eps_txt = f"{eps_t:4.2f} s" if eps_t is not None else "  -   "
        jump = max_step(all_cmds)
        print(f"rate {step.rate:2d} {step.direction:7s} -> {step.target:5d}: command {achieved:4.1f}/frame (max step {jump:3.0f}), " +
              f"EPS there in {eps_txt} {verdict}")
        log.write("sweep", rate=step.rate, direction=step.direction, target=step.target, achieved=achieved, max_step=jump,
                  eps_t=eps_t, verdict=verdict)
        last_of_rate = k + 1 == len(steps) or steps[k + 1].rate != step.rate
        if last_of_rate:
          results.append((step.rate, "ACCEPT"))
      elif step.direction != "rest":
        verdict = level_verdict(step.target, cmds, eps, False)
        results.append((step.level, verdict))
        cmd = statistics.median(abs(c) for c in cmds) if cmds else 0.
        eps_med = statistics.median(abs(e) for e in eps) if eps else 0.
        n += 1
        print(f"{n:2d}/{n_applies} {step.direction:5s} {step.level}: {verdict:11s} cmd {cmd:5.0f}  eps {eps_med:5.0f}")
        log.write("verdict", level=step.level, target=step.target, verdict=verdict, cmd=cmd, eps=eps_med)
    log.write("end")
    print("done")
    return 0
  except KeyboardInterrupt:
    abort = "stopped by hand"
    print("stopped")
    log.write("abort", reason=abort)
    return 1
  finally:
    for _ in range(30):
      publish(0.)
      time.sleep(DT)
    decision = sweep_summary(results, abort) if sweep else stage2_decision(stage, results, abort)
    if decision:
      print(decision)
      log.write("decision", decision=decision)
    lock.close()


def main() -> int:
  parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
  parser.add_argument("--stage", type=int, choices=(1, 2), required=True, help="1: standstill, 2: creeping at 1-3 km/h")
  parser.add_argument("--levels", type=int, nargs="+", help="torque levels (default: %(default)s)")
  parser.add_argument("--max", type=int, dest="max_level", help="stop at this level")
  parser.add_argument("--sweep", action="store_true", help="rate sweep instead of torque levels")
  parser.add_argument("--rates", type=int, nargs="+", help=f"rates per frame for --sweep (default: {DEFAULT_RATES})")
  parser.add_argument("--dry-run", action="store_true", help="print the schedule and exit")
  args = parser.parse_args()

  if args.sweep:
    try:
      steps = build_sweep(rates_from_args(args.rates))
    except ValueError as e:
      parser.error(str(e))
  else:
    steps = build_schedule(levels_from_args(args.levels, args.max_level))
  if args.dry_run:
    for s in steps:
      print(f"{s.direction:7s} target {s.target:6d}  rate {s.rate:2d}  {s.duration:4.2f} s")
    print(f"total {sum(s.duration for s in steps):.1f} s, stage {args.stage}")
    return 0
  ensure_repo_on_path(sys.path)
  return run(args.stage, steps, sweep=args.sweep)


if __name__ == "__main__":
  sys.exit(main())
