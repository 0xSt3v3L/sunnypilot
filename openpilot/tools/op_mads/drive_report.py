#!/usr/bin/env python3
"""Summarize an op-chestnut-mads route: MADS, standstill alerts, lane turns, road edges, blind spot and curves.

usage: drive_report.py <route dir with <segment>/rlog.zst>   (see pull_route.sh)
"""
import bisect
import math
import sys
from pathlib import Path

SERVICES = {"carParams", "carState", "carControl", "selfdriveState", "selfdriveStateSP", "onroadEvents", "radarState",
            "modelV2", "modelDataSP", "longitudinalPlan"}
STANDSTILL_ALERTS = ("greenLight", "leadDeparted")
MAX_DT = 0.1  # s, longer gaps between messages do not count as time
MIN_STOP = 2.  # s, the standstill alerts arm after 2 s
EVENT_DEDUPE = 1.  # s, onroadEvents repeat every second
TURN_WINDOW = 1.  # s of model curvature compared before and after a lane turn starts
CURVE_AY = 1.  # m/s^2 of lateral acceleration that makes a curve
CURVE_MIN_TIME = 1.
CURVE_GAP = 0.5  # s below CURVE_AY that still belongs to the same curve
CURVE_LOOKBACK = 5.  # s before the curve for the speed and model acceleration before it
CURVE_MIN_SPEED = 25. / 3.6  # m/s at the curve entry; slower turns are intersections
MS_TO_KPH = 3.6
HAS_BSM = 2  # ToyotaFlags.HAS_BSM


def _t(m) -> float:
  return m.logMonoTime * 1e-9


def _of(msgs, which):
  return [(_t(m), getattr(m, which)) for m in msgs if m.which() == which]


def _durations(series):
  """(t, value, dt) with dt to the next sample, capped at MAX_DT."""
  return [(t, v, min(series[i + 1][0] - t, MAX_DT) if i + 1 < len(series) else 0.) for i, (t, v) in enumerate(series)]


def _at(series, t):
  """Latest value at or before t, or None."""
  i = bisect.bisect_right([s[0] for s in series], t) - 1
  return series[i][1] if i >= 0 else None


def _window(series, t0, t1):
  return [v for t, v in series if t0 <= t < t1]


def _events(msgs, names):
  out: list[tuple[float, str]] = []
  for t, evs in _of(msgs, "onroadEvents"):
    for e in evs:
      name = str(e.name)
      if name in names and not any(n == name and t - tp < EVENT_DEDUPE for tp, n in out):
        out.append((t, name))
  return out


def _car_params(msgs):
  cps = _of(msgs, "carParams")
  return cps[0][1] if cps else None


def summary(msgs) -> dict:
  CP = _car_params(msgs)
  cs = _of(msgs, "carState")
  models = _of(msgs, "modelV2")
  times = [_t(m) for m in msgs]
  return {
    "fingerprint": CP.carFingerprint if CP else None,
    "has_bsm": bool(CP.flags & HAS_BSM) if CP and CP.brand == "toyota" else None,
    "mads_available": bool(CP.alternativeExperience & 1024) if CP else None,
    "duration_s": max(times) - min(times) if times else 0.,
    "distance_km": sum(c.vEgo * dt for _, c, dt in _durations(cs)) / 1000.,
    "big_ratio": sum(m.big for _, m in models) / len(models) if models else 0.,
  }


def mads_time(msgs) -> dict:
  ss = _of(msgs, "selfdriveState")
  cs = _of(msgs, "carState")
  full = lat_only = off = lat_moving = 0.
  for t, sp, dt in _durations(_of(msgs, "selfdriveStateSP")):
    s = _at(ss, t)
    if s is not None and s.enabled:
      full += dt
    elif sp.mads.active:
      lat_only += dt
    else:
      off += dt
  for t, cc, dt in _durations(_of(msgs, "carControl")):
    c = _at(cs, t)
    if cc.latActive and c is not None and c.vEgo > 1.:
      lat_moving += dt
  return {"full_s": full, "lat_only_s": lat_only, "off_s": off, "lat_moving_s": lat_moving,
          "mismatches": len(_events(msgs, ("controlsMismatchLateral",)))}


def standstill_report(msgs) -> dict:
  ss, radar, models = _of(msgs, "selfdriveState"), _of(msgs, "radarState"), _of(msgs, "modelV2")

  def lead_d(t):
    r = _at(radar, t)
    return r.leadOne.dRel if r is not None and r.leadOne.present else None

  def plan_x(t):
    m = _at(models, t)
    return m.position.x[-1] if m is not None and len(m.position.x) else None

  alerts = [{"t": t, "name": n, "lead_d": lead_d(t), "plan_x": plan_x(t)} for t, n in _events(msgs, STANDSTILL_ALERTS)]
  stops, start, moved = [], None, False
  cs = _of(msgs, "carState")
  for i, (t, c) in enumerate(cs):
    # like the alerts, ignore stops before the car first moved
    moved = moved or (not c.standstill and c.vEgo > 0.1)
    if c.standstill and start is None and moved:
      start = t
    if start is not None and (not c.standstill or i == len(cs) - 1):
      end = t
      s = _at(ss, start)
      if end - start >= MIN_STOP and not (s is not None and s.enabled):
        stops.append({"t": start, "duration": end - start, "lead": lead_d(start) is not None,
                      "alerted": any(start <= a["t"] <= end + 0.5 for a in alerts)})
      start = None
  return {"alerts": alerts, "stops": stops}


def lane_turn_report(msgs) -> list[dict]:
  cs = _of(msgs, "carState")
  curvature = [(t, m.action.desiredCurvature) for t, m in _of(msgs, "modelV2")]
  turns, prev = [], "none"
  for t, md in _of(msgs, "modelDataSP"):
    direction = str(md.laneTurnDirection)
    if direction != "none" and prev == "none":
      before, after = _window(curvature, t - TURN_WINDOW, t), _window(curvature, t, t + TURN_WINDOW)
      c = _at(cs, t)
      turns.append({"t": t, "direction": direction, "v_kph": c.vEgo * MS_TO_KPH if c is not None else None,
                    "curv_before": sum(before) / len(before) if before else None,
                    "curv_after": sum(after) / len(after) if after else None})
    prev = direction
  return turns


def road_edge_report(msgs) -> dict:
  edges = _of(msgs, "modelDataSP")
  left = sum(dt for _, md, dt in _durations(edges) if md.roadEdge.leftDetected)
  right = sum(dt for _, md, dt in _durations(edges) if md.roadEdge.rightDetected)
  cs = _of(msgs, "carState")
  blocked, was_blocked = 0, False
  for t, m in _of(msgs, "modelV2"):
    c, md = _at(cs, t), _at(edges, t)
    is_blocked = False
    if c is not None and md is not None and str(m.meta.laneChangeState) == "preLaneChange" and c.steeringPressed:
      direction = str(m.meta.laneChangeDirection)
      is_blocked = ((direction == "left" and c.steeringTorque > 0 and md.roadEdge.leftDetected) or
                    (direction == "right" and c.steeringTorque < 0 and md.roadEdge.rightDetected))
    blocked += is_blocked and not was_blocked
    was_blocked = is_blocked
  return {"left_s": left, "right_s": right, "blocked": blocked}


def bsm_report(msgs) -> dict:
  counts = {"left_events": 0, "right_events": 0}
  prev = (False, False)
  for _, c in _of(msgs, "carState"):
    counts["left_events"] += c.leftBlindspot and not prev[0]
    counts["right_events"] += c.rightBlindspot and not prev[1]
    prev = (c.leftBlindspot, c.rightBlindspot)
  return counts


def curve_report(msgs) -> list[dict]:
  CP = _car_params(msgs)
  if CP is None or CP.steerRatio == 0 or CP.wheelbase == 0:
    return []
  cs = _of(msgs, "carState")
  # lateral acceleration from the steering angle, as the longitudinal planner estimates it
  ay = [(t, c.vEgo ** 2 * math.radians(c.steeringAngleDeg) / (CP.steerRatio * CP.wheelbase)) for t, c in cs]
  episodes, start, last = [], None, None
  for t, a in ay:
    if abs(a) >= CURVE_AY:
      if start is not None and t - last > CURVE_GAP:
        episodes.append((start, last))
        start = None
      start = t if start is None else start
      last = t
  if start is not None:
    episodes.append((start, last))

  ss, plans = _of(msgs, "selfdriveState"), _of(msgs, "longitudinalPlan")
  model_accel = [(t, m.action.desiredAcceleration) for t, m in _of(msgs, "modelV2")]
  speed = [(t, c.vEgo) for t, c in cs]
  curves = []
  for t0, t1 in episodes:
    if t1 - t0 < CURVE_MIN_TIME or _at(speed, t0) < CURVE_MIN_SPEED:
      continue
    s, plan = _at(ss, t0), _at(plans, t0)
    before, accel = _at(speed, t0 - CURVE_LOOKBACK), _window(model_accel, t0 - CURVE_LOOKBACK, t0)
    curves.append({
      "t": t0, "duration": t1 - t0,
      "peak_ay": max(abs(a) for t, a in ay if t0 <= t <= t1),
      "v_before_kph": before * MS_TO_KPH if before is not None else None,
      "v_entry_kph": _at(speed, t0) * MS_TO_KPH,
      "v_min_kph": min(v for t, v in speed if t0 <= t <= t1) * MS_TO_KPH,
      "model_accel_min": min(accel) if accel else None,
      "engaged": bool(s is not None and s.enabled),
      "experimental": bool(s is not None and s.experimentalMode),
      "source": str(plan.longitudinalPlanSource) if plan is not None else None,
    })
  return curves


def _f(v, fmt):
  return "--" if v is None else format(v, fmt)


def format_report(msgs) -> str:
  t0 = min((_t(m) for m in msgs), default=0.)
  s, m = summary(msgs), mads_time(msgs)
  lines = ["== summary",
           f"  {s['fingerprint']}  {s['duration_s'] / 60:.1f} min  {s['distance_km']:.1f} km  big model {s['big_ratio'] * 100:.0f}%",
           f"  MADS available {s['mads_available']}  HAS_BSM flag {s['has_bsm']}",
           "== MADS",
           f"  full {m['full_s'] / 60:.1f} min  lateral-only {m['lat_only_s'] / 60:.1f} min  off {m['off_s'] / 60:.1f} min",
           f"  steering while moving {m['lat_moving_s'] / 60:.1f} min  lateral mismatches {m['mismatches']}"]

  st = standstill_report(msgs)
  alerted = sum(x["alerted"] for x in st["stops"])
  lines += ["== standstill alerts", f"  {len(st['stops'])} stops of {MIN_STOP:.0f}+ s without openpilot longitudinal, {alerted} with an alert"]
  lines += [f"  {a['t'] - t0:8.1f}s  {a['name']:13s} lead {_f(a['lead_d'], '.1f')} m  plan end {_f(a['plan_x'], '.0f')} m" for a in st["alerts"]]
  lines += [f"  {x['t'] - t0:8.1f}s  stop {x['duration']:5.1f}s  {'lead' if x['lead'] else 'no lead'}  no alert"
            for x in st["stops"] if not x["alerted"]]

  lines.append("== lane turns")
  lines += [f"  {x['t'] - t0:8.1f}s  {x['direction']:9s} {_f(x['v_kph'], '.0f')} km/h  model curvature " +
            f"{_f(x['curv_before'], '+.4f')} -> {_f(x['curv_after'], '+.4f')}" for x in lane_turn_report(msgs)] or ["  none"]

  r, b = road_edge_report(msgs), bsm_report(msgs)
  lines += ["== road edges", f"  detected left {r['left_s']:.0f}s  right {r['right_s']:.0f}s  lane changes blocked {r['blocked']}",
            "== blind spot", f"  left {b['left_events']}  right {b['right_events']}"]

  lines.append("== curves")
  lines += [f"  {c['t'] - t0:8.1f}s  peak {c['peak_ay']:.1f} m/s^2  {_f(c['v_before_kph'], '.0f')} -> {c['v_entry_kph']:.0f} km/h " +
            f"(min {c['v_min_kph']:.0f})  model accel min {_f(c['model_accel_min'], '+.2f')}  " +
            f"{'engaged' if c['engaged'] else 'not engaged'}{' experimental' if c['experimental'] else ''}  source {c['source']}"
            for c in curve_report(msgs)] or ["  none"]
  return "\n".join(lines)


def load_route(route_dir: str) -> list:
  from openpilot.tools.lib.logreader import LogReader
  segments = sorted((p for p in Path(route_dir).iterdir() if p.name.isdigit()), key=lambda p: int(p.name))
  msgs = [m for seg in segments for m in LogReader(str(seg / "rlog.zst")) if m.which() in SERVICES]
  return sorted(msgs, key=lambda m: m.logMonoTime)


if __name__ == "__main__":
  print(format_report(load_route(sys.argv[1])))
