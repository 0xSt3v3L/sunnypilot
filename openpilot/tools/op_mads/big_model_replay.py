#!/usr/bin/env python3
"""Replay the device's big driving model on the Mac from a route's full-resolution video.

usage: big_model_replay.py <route dir> [--segments 2-4] [--mode logged|ab|action_t] [--extras 0.3,0.6]

logged: the desire the device fed the model, rebuilt from the log, so the output can be checked against the logged
        modelV2 (the replay runs on the encoded video, so it is not exact).
ab:     two runs over the same frames, LaneTurnDesire off and on; each recomputes its desire with DesireHelper and RELC
        from its own outputs, as modeld does.
action_t: the logged desire, plus runs that ask the model for its lateral action that many seconds later (--extras)
        while below the lane turn speed with one blinker on, to see how much earlier a turn's curvature comes.
All print a modelV2 comparison and, for every low-speed blinker turn, the model curvature after the turn starts and
how long it takes to reach half the first run's peak.

Model files come from OP_MADS_MODELS (see its README). Missing fcamera.hevc / ecamera.hevc segments are copied from
the device (OP_MADS_DEVICE). Run from the test copy with
`DEV=METAL PYTHONPATH=<worktree>/opendbc_repo:$PWD:$PWD/tinygrad_repo .venv/bin/python <this script> ...`.
Times are seconds from the route start, as in drive_report.py.
"""
import argparse
import os
import time
from pathlib import Path

import numpy as np

from openpilot.cereal import log
from openpilot.selfdrive.controls.lib.desire_helper import TURN_DESIRES, DesireHelper
from openpilot.selfdrive.controls.lib.lane_turn import TurnDirection, lane_turn_direction
from openpilot.selfdrive.modeld.constants import ModelConstants
from openpilot.system.camerad.cameras.nv12_info import get_nv12_info
from openpilot.tools.op_mads.drive_report import _at, _of, route_segments, route_t0
from openpilot.tools.op_mads.frames import fetch_segment_file

MODELS = os.environ.get("OP_MADS_MODELS")
MODEL_PKL = "big_8d736683_metal.pkl"  # the device's big model, rebuilt; the warp pkls sit next to it
VIDEOS = {"narrowRoadEncodeIdx": "fcamera.hevc", "wideRoadEncodeIdx": "ecamera.hevc"}
SERVICES = {"carParams", "carState", "carControl", "deviceState", "narrowRoadCameraState", "extrinsicsCalibration",
            "driverMonitoringState", "lateralDelay", "modelV2", "modelDataSP", *VIDEOS}
WARMUP = 10.  # s after the replay starts while the model's history fills; not compared
TURN_COMPARE = 2.  # s of model curvature averaged after a lane turn starts
TURN_GAP = 1.5  # s without the blinker that still belongs to the same turn
TURN_NAMES = {TurnDirection.none: "none", TurnDirection.turnLeft: "turnLeft", TurnDirection.turnRight: "turnRight"}


def parse_segments(spec: str | None, available: list[int]) -> list[int]:
  if spec is None:
    return available
  lo, _, hi = spec.partition("-")
  wanted = list(range(int(lo), int(hi or lo) + 1))
  missing = [s for s in wanted if s not in available]
  if missing:
    raise SystemExit(f"segments without rlog.zst: {missing}")
  return wanted


def parse_extras(spec: str) -> list[float]:
  return [float(x) for x in spec.split(",") if x]


def extra_lat_action_t(car_state, extra: float) -> float:
  """extra seconds of lateral action time while a lane turn would be active (low speed, one blinker)."""
  return extra if lane_turn_direction(car_state, True) != TurnDirection.none else 0.


def onset_time(t: np.ndarray, curvature: np.ndarray, start: float, end: float, threshold: float) -> float | None:
  """First time in [start, end] where |curvature| reaches threshold, or None."""
  sel = (t >= start) & (t <= end) & (np.abs(curvature) >= threshold)
  return float(t[sel][0]) if sel.any() else None


def desire_from_state(turn_direction, lane_change_state, lane_change_direction) -> int:
  """DesireHelper.desire from its state, as modelV2.meta and modelDataSP log it (enums or their values)."""
  turn_direction, lane_change_state, lane_change_direction = (getattr(e, "raw", e) for e in
                                                              (turn_direction, lane_change_state, lane_change_direction))
  if turn_direction != TurnDirection.none:
    return TURN_DESIRES[turn_direction]
  if lane_change_state == log.LaneChangeState.laneChangeStarting:
    if lane_change_direction == log.LaneChangeDirection.left:
      return log.Desire.laneChangeLeft
    if lane_change_direction == log.LaneChangeDirection.right:
      return log.Desire.laneChangeRight
  return log.Desire.none


def desire_vector(desire: int) -> np.ndarray:
  vec = np.zeros(ModelConstants.DESIRE_LEN, dtype=np.float32)
  if 0 <= desire < ModelConstants.DESIRE_LEN:
    vec[desire] = 1
  return vec


def nv12_buffer(img: np.ndarray, w: int, h: int) -> np.ndarray:
  """A decoded (h * 3 / 2, w) NV12 frame in camerad's layout: rows padded to the stride, UV at y_height."""
  stride, y_height, uv_height, _ = get_nv12_info(w, h)
  buf = np.zeros(stride * (y_height + uv_height), dtype=np.uint8)
  planes = buf.reshape(y_height + uv_height, stride)
  planes[:h, :w] = img[:h]
  planes[y_height:y_height + h // 2, :w] = img[h:]
  return buf


class FrameSource:
  """Frames of one video by frameId, through its encode index; the video is read once, in order."""
  def __init__(self, frames, encode_idx: dict[int, int]):
    self.frames, self.encode_idx, self.pos, self.cur = frames, encode_idx, -1, None

  def get(self, frame_id: int):
    idx = self.encode_idx.get(frame_id)
    if idx is None or idx < self.pos:
      return None
    while self.pos < idx:
      self.cur, self.pos = next(self.frames), self.pos + 1
    return self.cur


def turn_name(direction) -> str:
  """A turn direction as a name, from an enum, its value or a name."""
  d = getattr(direction, "raw", direction)
  return TURN_NAMES.get(d, "none") if isinstance(d, int) else str(d)


def turn_events(series) -> list[tuple[float, float, str]]:
  """(start, end, direction) for each run of a direction other than none; a run that resumes the same direction
  within TURN_GAP is the same turn (the blinker signal drops out while waiting to turn)."""
  events, start, cur = [], None, "none"
  for t, d in series:
    d = turn_name(d)
    if d == cur:
      continue
    if cur != "none":
      events.append((start, t, cur))
    start, cur = t, d
    if cur != "none" and events and events[-1][2] == cur and t - events[-1][1] < TURN_GAP:
      start = events.pop()[0]
  if cur != "none" and series:
    events.append((start, series[-1][0], cur))
  return events


def compare_rows(keys, t, logged, replayed, skip: float) -> list[tuple[str, float, float, float]]:
  """(feature, correlation, rmse, logged std) over the samples at least skip seconds after the first."""
  sel = t >= t[0] + skip
  rows = []
  for i, k in enumerate(keys):
    a, b = logged[sel, i], replayed[sel, i]
    sd = float(np.std(a))
    corr = float(np.corrcoef(a, b)[0, 1]) if sd > 0 and np.std(b) > 0 else float("nan")
    rows.append((k, corr, float(np.sqrt(np.mean((a - b) ** 2))), sd))
  return rows


def features(mv2) -> dict[str, float]:
  """The modelV2 values compared, one float each (T_IDXS[14] ~ 1.9 s, T_IDXS[20] ~ 3.9 s, X_IDXS[14] ~ 37 m)."""
  out = {
    "desiredCurvature": mv2.action.desiredCurvature,
    "desiredAcceleration": mv2.action.desiredAcceleration,
    "path_y_2s": mv2.position.y[14],
    "path_x_4s": mv2.position.x[20],
    "laneLine_left_y": mv2.laneLines[1].y[14],
    "laneLine_right_y": mv2.laneLines[2].y[14],
    "roadEdge_left_y": mv2.roadEdges[0].y[14],
    "roadEdge_right_y": mv2.roadEdges[1].y[14],
    "lead_x": mv2.leadsV3[0].x[0],
    "lead_prob": mv2.leadsV3[0].prob,
  }
  return {k: float(v) for k, v in out.items()}


def nv12_frames(path: Path, w: int, h: int):
  import av
  with av.open(str(path), format="hevc") as container:
    for frame in container.decode(video=0):
      yield nv12_buffer(frame.to_ndarray(format="nv12"), w, h)


class Camera:
  """Frames of one camera across segments, copied from the device when missing."""
  def __init__(self, route: Path, encode_idx, w: int, h: int):
    self.route, self.video, self.w, self.h = route, VIDEOS[encode_idx[0]], w, h
    self.where = {e.frameId: (e.segmentNum, e.segmentId) for _, e in encode_idx[1]}
    self.seg, self.src = None, None

  def get(self, frame_id: int):
    seg = self.where.get(frame_id, (None,))[0]
    if seg is None:
      return None
    if seg != self.seg:
      idx = {f: i for f, (s, i) in self.where.items() if s == seg}
      self.seg, self.src = seg, FrameSource(nv12_frames(fetch_segment_file(self.route, seg, self.video), self.w, self.h), idx)
    return self.src.get(frame_id)


class Buf:
  def __init__(self, data):
    self.data = data


class Run:
  """One model instance with its own desire state; lane_turn None feeds the logged desire."""
  def __init__(self, name: str, model, lane_turn: bool | None, action_t_extra: float = 0., extra_inputs=None):
    from openpilot.selfdrive.controls.lib.relc import RoadEdgeLaneChangeController
    from openpilot.common.realtime import DT_MDL
    from openpilot.selfdrive.controls.lib.low_speed_lat import LowSpeedLatShaper
    from openpilot.selfdrive.modeld.fill_model_msg import PublishState
    self.name, self.model, self.lane_turn, self.action_t_extra = name, model, lane_turn, action_t_extra
    self.extra_inputs = extra_inputs  # t -> more model inputs, such as the navigation ones
    self.dh, self.relc, self.publish_state = DesireHelper(), RoadEdgeLaneChangeController(), PublishState()
    self.prev_action = log.ModelDataV2.Action()
    self.lat_shaper = LowSpeedLatShaper(DT_MDL)
    self.rows: list[dict[str, float]] = []

  def step(self, md, bufs, transforms, desire, inputs, v_ego, lat_action_t, long_action_t, logged, car_state, lat_active,
           t: float = 0.):
    from openpilot.cereal import messaging
    from openpilot.selfdrive.modeld.fill_model_msg import fill_model_msg
    if self.lane_turn is not None:
      desire = self.dh.desire
    lat_action_t += extra_lat_action_t(car_state, self.action_t_extra)
    inputs = inputs | {"desire_pulse": desire_vector(desire),
                       "action_t": np.array([lat_action_t, long_action_t], dtype=np.float32)}
    if self.extra_inputs is not None:
      inputs |= self.extra_inputs(t)
    out = self.model.run(bufs, transforms, inputs)
    self.prev_action = md.get_action_from_model(out, self.prev_action, lat_action_t, long_action_t, v_ego, self.lat_shaper)
    msg = messaging.new_message("modelV2")
    fill_model_msg(msg, out, self.prev_action, self.publish_state, logged.frameId, logged.frameIdExtra, logged.frameId,
                   0., logged.timestampEof, 0., True)
    mv2 = msg.modelV2
    if self.lane_turn is not None:
      prob = mv2.meta.desireState[log.Desire.laneChangeLeft] + mv2.meta.desireState[log.Desire.laneChangeRight]
      left_edge, right_edge = self.relc.update(mv2.roadEdgeStds, mv2.laneLineProbs, v_ego, mv2.roadEdges)
      self.dh.update(car_state, lat_active, prob, left_edge, right_edge, lane_turn_enabled=self.lane_turn)
    self.rows.append(features(mv2))


def load_model(md, models: Path, w: int, h: int, pkl: str = MODEL_PKL):
  from tinygrad import Context
  from tinygrad_repo.examples.openpilot.helpers import load_pickle

  def load_metal(path, chestnut=False):
    with Context(DEV="METAL"):
      return load_pickle(models / pkl, out_of_band=True)

  md.load_oob, md.MODELS_DIR = load_metal, models
  model = md.ModelState(w, h, True)
  model.warmup()
  return model


def mode_runs(mode: str, extras=(0.3, 0.6)):
  """make_runs for replay(): the runs of a --mode."""
  def make_runs(load, of):
    if mode == "logged":
      return [Run("replay", load(), None)]
    if mode == "ab":
      return [Run("lane turn off", load(), False), Run("lane turn on", load(), True)]
    return [Run("baseline", load(), None)] + [Run(f"action_t +{e:g}s", load(), None, e) for e in extras]
  return make_runs


def replay(route: Path, segments: list[int], make_runs, models: Path, services=SERVICES):
  """make_runs(load, of) gives the runs; load(pkl=MODEL_PKL) loads a model, of holds the messages by service."""
  from openpilot.common.realtime import DT_MDL
  from openpilot.common.transformations.camera import DEVICE_CAMERAS
  from openpilot.common.transformations.model import get_warp_matrix
  from openpilot.selfdrive.modeld import modeld as md
  from openpilot.tools.lib.logreader import LogReader

  rlogs, _ = route_segments(str(route))
  rlogs = [p for p in rlogs if int(p.parent.name) in segments]
  msgs = sorted((m for p in rlogs for m in LogReader(str(p)) if m.which() in services), key=lambda m: m.logMonoTime)
  of = {s: _of(msgs, s) for s in services}
  CP = of["carParams"][0][1]
  dc = DEVICE_CAMERAS[(str(of["deviceState"][0][1].deviceType), str(of["narrowRoadCameraState"][0][1].sensor))]
  w, h = dc.narrow_road.width, dc.narrow_road.height
  main, extra = (Camera(route, (k, of[k]), w, h) for k in VIDEOS)

  runs = make_runs(lambda pkl=MODEL_PKL: load_model(md, models, w, h, pkl), of)
  long_action_t = CP.longitudinalActuatorDelay + md.LONG_SMOOTH_SECONDS + DT_MDL + DT_MDL / 2
  times, speeds, logged_rows, turns, run_ms = [], [], [], [], []
  prev_mv = None

  for t, mv in of["modelV2"]:
    prev_sp = _at(of["modelDataSP"], t)
    desire = log.Desire.none if prev_mv is None or prev_sp is None else \
      desire_from_state(prev_sp.laneTurnDirection, prev_mv.meta.laneChangeState, prev_mv.meta.laneChangeDirection)
    prev_mv = mv
    rpy, cs = _at(of["extrinsicsCalibration"], t), _at(of["carState"], t)
    bufs = {"img": main.get(mv.frameId), "big_img": extra.get(mv.frameIdExtra)}
    if not mv.big or rpy is None or cs is None or any(b is None for b in bufs.values()):
      continue
    rpy = np.array(rpy.rpyCalib, dtype=np.float32)
    transforms = {"img": get_warp_matrix(rpy, dc.narrow_road.intrinsics, False).astype(np.float32),
                  "big_img": get_warp_matrix(rpy, dc.wide_road.intrinsics, True).astype(np.float32)}
    dms, lat, cc = _at(of["driverMonitoringState"], t), _at(of["lateralDelay"], t), _at(of["carControl"], t)
    traffic_convention = np.zeros(2)
    traffic_convention[int(dms is not None and dms.isRHD)] = 1
    lat_action_t = (lat.lateralDelay if lat is not None else 0.) + md.LAT_SMOOTH_SECONDS + DT_MDL + DT_MDL / 2
    inputs = {"traffic_convention": traffic_convention}
    v_ego = max(cs.vEgo, 0.)
    st = time.perf_counter()
    for r in runs:
      r.step(md, {k: Buf(b) for k, b in bufs.items()}, transforms, desire, inputs, v_ego, lat_action_t, long_action_t,
             mv, cs, cc is not None and cc.latActive, t)
    run_ms.append((time.perf_counter() - st) * 1e3 / len(runs))
    times.append(t)
    speeds.append(v_ego)
    logged_rows.append(features(mv))
    turns.append((t, lane_turn_direction(cs, True)))
  return times, speeds, logged_rows, runs, turns, run_ms


def report(t0: float, times, speeds, logged_rows, runs, turns, run_ms) -> str:
  keys = list(logged_rows[0])
  t = np.array(times)
  logged = np.array([[r[k] for k in keys] for r in logged_rows])
  lines = [f"{len(t)} frames, {t[0] - t0:.1f}-{t[-1] - t0:.1f}s, compared after {WARMUP:.0f}s; " +
           f"model run median {np.median(run_ms):.1f} ms"]
  for r in runs:
    rep = np.array([[row[k] for k in keys] for row in r.rows])
    lines += ["", f"{r.name} vs logged", f"  {'feature':<22}{'corr':>8}{'rmse':>10}{'log std':>10}{'rmse/std':>10}"]
    for k, corr, rmse, sd in compare_rows(keys, t, logged, rep, WARMUP):
      lines.append(f"  {k:<22}{corr:>8.4f}{rmse:>10.4f}{sd:>10.4f}{(rmse / sd if sd > 0 else float('nan')):>10.3f}")

  ci = keys.index("desiredCurvature")
  curv = [("logged", logged[:, ci])] + [(r.name, np.array([row["desiredCurvature"] for row in r.rows])) for r in runs]
  lines += ["", f"blinker turns below the lane turn speed: mean desiredCurvature over the first {TURN_COMPARE:.0f}s, " +
            f"peak |desiredCurvature| (1/m) and seconds to reach half of {runs[0].name}'s peak; * = during warm-up"]
  lines.append(f"  {'start':>8} {'dir':<10}{'km/h':>6}" + "".join(f"{name:>33}" for name, _ in curv))
  for start, end, direction in turn_events(turns):
    first = (t >= start) & (t < start + TURN_COMPARE)
    whole = (t >= start) & (t <= end)
    kph = speeds[int(np.searchsorted(t, start))] * 3.6
    half = np.abs(curv[1][1][whole]).max() / 2
    onsets = [onset_time(t, c, start, end, half) for _, c in curv]
    cells = "".join(f"{c[first].mean():>+13.4f}{np.abs(c[whole]).max():>13.4f}" +
                    (f"{o - start:>7.1f}" if o is not None else f"{'--':>7}") for (_, c), o in zip(curv, onsets, strict=True))
    mark = "*" if start < t[0] + WARMUP else " "
    lines.append(f"  {start - t0:>7.1f}{mark} {direction:<10}{kph:>6.0f}{cells}")
  return "\n".join(lines)


def main() -> None:
  parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
  parser.add_argument("route_dir")
  parser.add_argument("--segments", help="e.g. 2-4; default: all")
  parser.add_argument("--mode", choices=["logged", "ab", "action_t"], default="logged")
  parser.add_argument("--extras", default="0.3,0.6", help="action_t mode: extra seconds, comma separated")
  parser.add_argument("--models", default=MODELS, help="default: OP_MADS_MODELS")
  args = parser.parse_args()
  if not args.models:
    raise SystemExit("set OP_MADS_MODELS or pass --models")
  route = Path(args.route_dir)
  rlogs, _ = route_segments(str(route))
  segments = parse_segments(args.segments, [int(p.parent.name) for p in rlogs])
  out = replay(route, segments, mode_runs(args.mode, parse_extras(args.extras)), Path(args.models))
  print(report(route_t0(str(route)), *out))


if __name__ == "__main__":
  main()
