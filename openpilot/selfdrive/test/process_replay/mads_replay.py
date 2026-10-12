#!/usr/bin/env python3
"""Replay selfdrived on a local route and check MADS against the logged run.

usage: mads_replay.py <route dir with <segment>/rlog.zst>
The route data stays outside the repo (it is large and holds GPS positions).
"""
import sys
from pathlib import Path

STATE_TOLERANCE = 0.05  # s between a logged and a replayed transition
WARMUP = 1.  # s after selfdrived finished initializing before transitions are compared
STANDSTILL_ALERTS = ("greenLight", "leadDeparted")


def transitions(msgs, which: str, value) -> list[tuple[float, object]]:
  out: list[tuple[float, object]] = []
  for m in msgs:
    if m.which() == which:
      v = value(getattr(m, which))
      if not out or out[-1][1] != v:
        out.append((m.logMonoTime * 1e-9, v))
  return out


def mads_state(ss_sp) -> str:
  return str(ss_sp.mads.state)


def enabled(ss) -> bool:
  return bool(ss.enabled)


def _event_names(m) -> list[str]:
  return [str(e.name) for e in m.onroadEvents]


def init_end_time(msgs) -> float:
  return max((m.logMonoTime * 1e-9 for m in msgs if m.which() == "onroadEvents" and "selfdriveInitializing" in _event_names(m)), default=0.)


def compare_transitions(logged, replayed, start: float, tol: float = STATE_TOLERANCE) -> list[str]:
  logged = [x for x in logged if x[0] >= start]
  replayed = [x for x in replayed if x[0] >= start]
  if [v for _, v in logged] != [v for _, v in replayed]:
    i = next((i for i, (a, b) in enumerate(zip(logged, replayed, strict=False)) if a[1] != b[1]), min(len(logged), len(replayed)))
    return [f"sequence differs at #{i}: logged {logged[i:i + 3]} replayed {replayed[i:i + 3]}"]
  return [f"{v} at {lt:.2f}s logged, {rt:.2f}s replayed" for (lt, v), (rt, _) in zip(logged, replayed, strict=True) if abs(lt - rt) > tol]


def mismatch_events(msgs) -> list[float]:
  return [m.logMonoTime * 1e-9 for m in msgs if m.which() == "onroadEvents" and "controlsMismatchLateral" in _event_names(m)]


def standstill_alert_violations(msgs) -> list[str]:
  """Standstill alerts may only fire while stopped and openpilot does not control longitudinal."""
  car_state = selfdrive_state = None
  problems = []
  for m in msgs:
    w = m.which()
    if w == "carState":
      car_state = m.carState
    elif w == "selfdriveState":
      selfdrive_state = m.selfdriveState
    elif w == "onroadEvents":
      for name in set(_event_names(m)) & set(STANDSTILL_ALERTS):
        t = m.logMonoTime * 1e-9
        if car_state is None or not car_state.standstill:
          problems.append(f"{name} while moving at {t:.2f}s")
        elif selfdrive_state is not None and selfdrive_state.enabled:
          problems.append(f"{name} while engaged at {t:.2f}s")
  return problems


def load_route(route_dir: str, keep: set[str]) -> list:
  from openpilot.tools.lib.logreader import LogReader
  segments = sorted((p for p in Path(route_dir).iterdir() if p.name.isdigit()), key=lambda p: int(p.name))
  return [m for seg in segments for m in LogReader(str(seg / "rlog.zst")) if m.which() in keep]


def run_route(route_dir: str) -> dict:
  from openpilot.selfdrive.test.process_replay.process_replay import get_process_config, replay_process
  cfg = get_process_config("selfdrived")
  outputs = set(cfg.subs)
  msgs = load_route(route_dir, set(cfg.pubs) | outputs | {"carParams", "initData", "can"})
  logged = [m for m in msgs if m.which() in outputs | {"carState"}]
  replayed = replay_process(cfg, [m for m in msgs if m.which() not in outputs], disable_progress=True)
  replayed_stream = sorted(replayed + [m for m in msgs if m.which() == "carState"], key=lambda m: m.logMonoTime)

  start = max(init_end_time(logged), init_end_time(replayed)) + WARMUP
  problems = []
  for which, value in (("selfdriveStateSP", mads_state), ("selfdriveState", enabled)):
    problems += [f"{which}: {p}" for p in compare_transitions(transitions(logged, which, value), transitions(replayed, which, value), start)]
  problems += [f"controlsMismatchLateral at {t:.2f}s" for t in mismatch_events(replayed)]
  problems += standstill_alert_violations(replayed_stream)
  return {
    "start": start,
    "mads_transitions": transitions(replayed, "selfdriveStateSP", mads_state),
    "standstill_alerts": [(m.logMonoTime * 1e-9, n) for m in replayed if m.which() == "onroadEvents"
                          for n in _event_names(m) if n in STANDSTILL_ALERTS],
    "problems": problems,
  }


def main() -> int:
  report = run_route(sys.argv[1])
  t0 = report["mads_transitions"][0][0] if report["mads_transitions"] else 0.
  print(f"compared from {report['start'] - t0:.1f}s")
  for t, state in report["mads_transitions"]:
    print(f"  {t - t0:8.2f}s  mads {state}")
  for t, name in report["standstill_alerts"]:
    print(f"  {t - t0:8.2f}s  {name}")
  for p in report["problems"]:
    print(f"PROBLEM {p}")
  print("PASS" if not report["problems"] else "FAIL")
  return 1 if report["problems"] else 0


if __name__ == "__main__":
  sys.exit(main())
