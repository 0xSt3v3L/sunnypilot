#!/usr/bin/env python3
"""Replay comma's navigation driving model on the Mac with a Google route along the driven path.

usage: nav_model_replay.py <route dir> --segments 2-4 --model <nav model pkl> --tiles <PMTiles URL> [--maps out.png]
       [--ablation]

Google routes from the first to the last GPS fix of the segments, through up to 25 via points spread along the driven
path, so the route follows the drive. Two runs of the nav model share the logged desire: "nav off" with zero map and
navigation inputs, "nav on" with the inputs mapsd and navd would give it (the map rendered from the PMTiles basemap,
centered on the route ahead of the last fix; the next maneuver as a Valhalla command and its distance). For each turn
maneuver of the route it prints the curvature (positive to the right) of the logged device model and of both runs.
--maps saves the map input at each maneuver. --ablation adds runs with the map alone, the instruction alone, and both
with the instruction mirrored left/right, to see which input the model follows.

The Google API key comes from GOOGLE_MAPS_API_KEY or ~/.config/op-mads/google_maps_api_key. --tiles is a PMTiles v3
Protomaps basemap (for example a build from build.protomaps.com); tiles along the route are cached under
~/.comma/navigation/tiles. Run from the test copy with
`DEV=METAL PYTHONPATH=<worktree>/opendbc_repo:$PWD:$PWD/tinygrad_repo .venv/bin/python <this script> ...`.
Times are seconds from the route start, as in drive_report.py.
"""
import argparse
import os
from pathlib import Path
from types import SimpleNamespace

import numpy as np

from openpilot.selfdrive.navigation.helpers import navigation_input, valhalla_type
from openpilot.selfdrive.navigation.renderer import SIZE
from openpilot.selfdrive.navigation.route import RouteTracker, along_path, cumulative_distance, local_meters
from openpilot.tools.op_mads.big_model_replay import MODELS, SERVICES, Run, WARMUP, parse_segments, replay
from openpilot.tools.op_mads.drive_report import _of, route_segments, route_t0

GPS_SERVICES = ("gpsLocationExternal", "gpsLocation")
KEY_FILE = Path.home() / ".config" / "op-mads" / "google_maps_api_key"
MAX_VIAS = 25  # Google's limit on intermediate waypoints
VIA_SPACING = 400.  # m; the longest stretch of the track without a via
TURN_ANGLE = 45.  # degrees of heading change across TURN_WINDOW that make a turn
TURN_WINDOW = 30.  # m before and after a point
TURN_VIA_OFFSET = 40.  # m past a turn's middle, where the via goes
HEADING_CHORD = 15.  # m behind and ahead of a point for its heading
SAMPLE = 5.  # m between the track samples searched for turns
FIX_TIMEOUT = 2.  # s; navd and mapsd drop a fix this old
REROUTE_DISTANCE = 25.  # m off the route, as navd
REROUTE_COUNT = 3  # navd reroutes after more than this many 1 Hz updates off the route
VIA_LOOKAHEAD = 30.  # m; a reroute keeps the vias at least this far ahead along the track
# Valhalla commands and their mirror images: slight, plain and sharp turns, on ramps, exits and forks
FLIPPED = {9: 16, 10: 15, 11: 14, 18: 19, 20: 21, 23: 24}
FLIPPED |= {v: k for k, v in FLIPPED.items()}
EVENT_WINDOW = (-3., 2.)  # s around passing a maneuver over which curvature is averaged


def api_key(path: Path = KEY_FILE) -> str:
  key = os.environ.get("GOOGLE_MAPS_API_KEY")
  if key:
    return key
  try:
    return path.read_text().strip()
  except FileNotFoundError:
    raise SystemExit(f"set GOOGLE_MAPS_API_KEY or put the key in {path}") from None


def gps_fixes(of) -> np.ndarray:
  """(t, longitude, latitude, speed, bearing) rows of the GPS service with the most fixes."""
  best = np.empty((0, 5))
  for service in GPS_SERVICES:
    rows = np.array([(t, g.longitude, g.latitude, g.speed, g.bearingDeg) for t, g in of.get(service, []) if g.hasFix],
                    dtype=np.float64).reshape((-1, 5))
    if len(rows) > len(best):
      best = rows
  return best


def via_points(track: np.ndarray, spacing: float = VIA_SPACING, max_count: int = MAX_VIAS) -> np.ndarray:
  """(longitude, latitude, heading, meters along the track) vias that keep Google on a driven (longitude, latitude) track: one just past each
  turn, so every turn is taken as driven, and evenly spaced ones on stretches longer than spacing. With its heading a
  via is not reached the other way round with a U-turn; GPS points a few meters off the road need no exact detour."""
  if len(track) < 2:
    return np.empty((0, 4))
  distance = cumulative_distance(track)
  total = distance[-1]
  if total <= 0:
    return np.empty((0, 4))

  def points(d):
    d = np.clip(d, 0, total)
    return np.column_stack([np.interp(d, distance, track[:, axis]) for axis in (0, 1)])

  def heading(d):
    (lon0, lat0), (lon1, lat1) = points(d - HEADING_CHORD).T, points(d + HEADING_CHORD).T
    return np.degrees(np.arctan2((lon1 - lon0) * np.cos(np.radians(lat0)), lat1 - lat0)) % 360

  d = np.arange(0, total, SAMPLE)
  change = np.abs((heading(d + TURN_WINDOW) - heading(d - TURN_WINDOW) + 180) % 360 - 180)
  turning = change > TURN_ANGLE
  starts = np.flatnonzero(turning & ~np.r_[False, turning[:-1]])
  ends = np.flatnonzero(turning & ~np.r_[turning[1:], False])
  anchors = [d[a + np.argmax(change[a:b + 1])] + TURN_VIA_OFFSET for a, b in zip(starts, ends, strict=True)]
  anchors = [a for a in anchors if a < total]
  at = []
  for lo, hi in zip([0.] + anchors, anchors + [total], strict=True):
    n = int(np.ceil((hi - lo) / spacing)) - 1
    at += list(np.linspace(lo, hi, n + 2)[1:-1]) + ([hi] if hi < total else [])
  if len(at) > max_count:
    raise ValueError(f"the track needs {len(at)} vias, Google takes {max_count}: replay fewer segments")
  at = np.array(at)
  return np.column_stack((points(at), heading(at), at)) if len(at) else np.empty((0, 4))


def route_offsets(track: np.ndarray, route: np.ndarray) -> np.ndarray:
  """Meters from each track point to the route."""
  return np.array([along_path(route, point)[1] for point in track])


class NavInputs:
  """The map and navigation inputs over a replay, as mapsd and navd give them from the logged GPS fixes.

  router(fix index) gives the steps of a route from that fix; like navd, a route is requested again after more than
  REROUTE_COUNT seconds over REROUTE_DISTANCE off it."""

  def __init__(self, router, fixes: np.ndarray, renderer):
    self.router, self.fixes, self.renderer = router, fixes, renderer
    self.events: list[tuple[float, int, str, str]] = []  # (time passed, Valhalla command, side, instruction)
    self.maps: list[np.ndarray] = []  # the map input at each maneuver passed
    self.reroutes, self.offroute, self.checked = 0, 0, -np.inf
    self.progress = -np.inf
    self.last: tuple[float, dict] | None = None  # (t, inputs): runs over the same frame share one update
    self.set_route(router(0))

  def set_route(self, steps: list[dict]):
    geometry = [np.asarray(s['geometry']['coordinates'], dtype=np.float64).reshape((-1, 2)) for s in steps]
    self.steps = steps
    self.route = np.concatenate(geometry)
    self.tracker = RouteTracker(self.route)
    self.tracker.progress = 0.  # a route starts at the car, so a loop back past its start cannot capture the first fix
    # each step begins with its maneuver, measured along the route as navd does
    self.maneuver_distances = self.tracker.distance[np.cumsum([0] + [len(g) for g in geometry[:-1]])]
    self.passed = {0}

  def maneuvers(self, progress: float):
    return [SimpleNamespace(distance=float(d - progress), type=s['maneuver']['type'], modifier=s['maneuver'].get('modifier', ''))
            for s, d in zip(self.steps, self.maneuver_distances, strict=True) if d >= progress]

  def check_route(self, i: int):
    """navd's 1 Hz off-route check at fix i."""
    lon, lat = self.fixes[i, 1:3]
    on_route = self.tracker.pose(self.tracker.project(lon, lat))[:2]
    self.offroute = self.offroute + 1 if np.linalg.norm(local_meters((lon, lat), on_route)) > REROUTE_DISTANCE else 0
    if self.offroute > REROUTE_COUNT:
      steps = self.router(i)
      if steps:
        self.set_route(steps)
        self.reroutes += 1
      self.offroute = 0

  def passed_maneuver(self, k: int, t: float, nav_map: np.ndarray):
    self.passed.add(k)
    self.maps.append(nav_map)
    m = self.steps[k]['maneuver']
    command = valhalla_type(m['type'], m.get('modifier', ''))
    if command is None or command <= 6 or command == 8:
      return
    side = 'right' if 'right' in m.get('modifier', '') else 'left' if 'left' in m.get('modifier', '') else ''
    banner = self.steps[k - 1].get('bannerInstructions') or [{}]
    self.events.append((float(t), command, side, banner[0].get('primary', {}).get('text', '')))

  def at(self, t: float) -> dict[str, np.ndarray]:
    if self.last is None or self.last[0] != t:
      self.last = (t, self.update(t))
    return self.last[1]

  def update(self, t: float) -> dict[str, np.ndarray]:
    i = int(np.searchsorted(self.fixes[:, 0], t, side='right')) - 1
    if i < 0 or t - self.fixes[i, 0] >= FIX_TIMEOUT:
      return {}
    fix_t, lon, lat, speed, _ = self.fixes[i]
    if fix_t - self.checked >= 1.:
      self.checked = fix_t
      self.check_route(i)
    self.progress = self.tracker.project(lon, lat)
    ahead = max(speed, 0.) * (t - fix_t)
    navigation = navigation_input(self.maneuvers(self.progress), ahead)
    image, valid = self.renderer.render(*self.tracker.pose(self.progress + ahead), self.route)
    nav_map = image if valid else np.zeros((SIZE, SIZE), dtype=np.uint8)  # mapsd sends black without a usable map
    for k, d in enumerate(self.maneuver_distances):
      if k not in self.passed and self.progress + ahead >= d:
        self.passed_maneuver(k, t, nav_map)
    return {'navigation': navigation, 'nav_map': nav_map}


def flip_navigation(navigation: np.ndarray) -> np.ndarray:
  """The navigation input with the maneuver's side mirrored, its distance kept."""
  out = navigation.copy()
  for command in np.flatnonzero(navigation[:-1]):
    if command in FLIPPED:
      out[command], out[FLIPPED[command]] = 0, navigation[command]
  return out


def ablation_inputs(nav: NavInputs) -> dict:
  """Extra inputs per run name: the map alone, the instruction alone, both, and both with the instruction mirrored."""
  def only(name):
    return lambda t: {k: v for k, v in nav.at(t).items() if k == name}

  def flipped(t):
    inputs = nav.at(t)
    return inputs | {'navigation': flip_navigation(inputs['navigation'])} if inputs else {}

  return {'map only': only('nav_map'), 'instruction only': only('navigation'), 'nav on': nav.at, 'nav flipped': flipped}


def report(t0, times, speeds, logged_rows, runs, nav, fixes, steps) -> str:
  t = np.array(times)
  track = fixes[:, 1:3]
  first = np.concatenate([np.asarray(s['geometry']['coordinates'], dtype=np.float64).reshape((-1, 2)) for s in steps])
  offsets = route_offsets(track, first)
  driven = cumulative_distance(track)[-1]
  lines = [f"{len(t)} frames, {t[0] - t0:.1f}-{t[-1] - t0:.1f}s; first route {cumulative_distance(first)[-1]:.0f} m in " +
           f"{len(steps)} steps, driven {driven:.0f} m; GPS to it median {np.median(offsets):.1f} m, " +
           f"p95 {np.percentile(offsets, 95):.1f} m, max {offsets.max():.1f} m; {nav.reroutes} reroutes"]
  curv = [("logged", np.array([r["desiredCurvature"] for r in logged_rows]))] + \
         [(r.name, np.array([row["desiredCurvature"] for row in r.rows])) for r in runs]
  lines += ["", f"turn maneuvers: mean / signed peak desiredCurvature (1/m, + = right) over {EVENT_WINDOW[0]:+.0f} to " +
            f"{EVENT_WINDOW[1]:+.0f} s around passing them; * = during warm-up",
            f"  {'time':>8} {'cmd':>3} {'side':<6}{'km/h':>6}" + "".join(f"{name:>22}" for name, _ in curv) + "  instruction"]
  for when, command, side, text in nav.events:
    sel = (t >= when + EVENT_WINDOW[0]) & (t <= when + EVENT_WINDOW[1])
    if not sel.any():
      continue
    kph = speeds[min(int(np.searchsorted(t, when)), len(speeds) - 1)] * 3.6
    cells = "".join(f"{c[sel].mean():>+11.4f}{c[sel][np.abs(c[sel]).argmax()]:>+11.4f}" for _, c in curv)
    mark = "*" if when < t[0] + WARMUP else " "
    lines.append(f"  {when - t0:>7.1f}{mark} {command:>3} {side:<6}{kph:>6.0f}{cells}  {text}")
  return "\n".join(lines)


def save_maps(nav: NavInputs, path: Path):
  from PIL import Image
  if nav.maps:
    Image.fromarray(np.hstack(nav.maps)).save(path)


def main() -> None:
  from openpilot.selfdrive.navigation.google import compute_route
  from openpilot.selfdrive.navigation.pmtiles import PMTiles
  from openpilot.selfdrive.navigation.renderer import MapRenderer
  from openpilot.selfdrive.navigation.tiles import RemoteTiles
  from openpilot.tools.lib.logreader import LogReader

  parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
  parser.add_argument("route_dir")
  parser.add_argument("--segments", help="e.g. 2-4; default: all")
  parser.add_argument("--model", required=True, help="the nav model's METAL pkl, in --models")
  parser.add_argument("--models", default=MODELS, help="default: OP_MADS_MODELS")
  parser.add_argument("--tiles", required=True, help="PMTiles basemap URL")
  parser.add_argument("--maps", type=Path, help="save the map input at each maneuver to this PNG")
  parser.add_argument("--ablation", action="store_true",
                      help="also run the map alone, the instruction alone and the instruction mirrored left/right")
  args = parser.parse_args()
  if not args.models:
    raise SystemExit("set OP_MADS_MODELS or pass --models")
  route = Path(args.route_dir)
  rlogs, _ = route_segments(str(route))
  segments = parse_segments(args.segments, [int(p.parent.name) for p in rlogs])

  gps = sorted((m for p in rlogs if int(p.parent.name) in segments for m in LogReader(str(p)) if m.which() in GPS_SERVICES),
               key=lambda m: m.logMonoTime)
  fixes = gps_fixes({s: _of(gps, s) for s in GPS_SERVICES})
  if len(fixes) < 2:
    raise SystemExit("no GPS fixes in these segments")
  track, key = fixes[:, 1:3], api_key()
  vias, along = via_points(track), cumulative_distance(track)
  renderer = MapRenderer(PMTiles(RemoteTiles(url=args.tiles)))
  routes = []

  def router(i):
    ahead = vias[vias[:, 3] > along[i] + VIA_LOOKAHEAD, :3]
    steps = compute_route(track[i], track[-1], key, heading=fixes[i, 4], vias=ahead)
    if steps:
      routes.append(steps)
      renderer.prefetch(np.concatenate([np.asarray(s['geometry']['coordinates']).reshape((-1, 2)) for s in steps]))
    return steps

  first = router(0)
  if not first:
    raise SystemExit("Google returned no route")
  nav = NavInputs(lambda i: first if i == 0 else router(i), fixes, renderer)

  def make_runs(load, of):
    extra = ablation_inputs(nav) if args.ablation else {"nav on": nav.at}
    return [Run("nav off", load(args.model), None)] + [Run(name, load(args.model), None, extra_inputs=f) for name, f in extra.items()]

  times, speeds, logged_rows, runs, _, _ = replay(route, segments, make_runs, Path(args.models), SERVICES | set(GPS_SERVICES))
  print(report(route_t0(str(route)), times, speeds, logged_rows, runs, nav, fixes, routes[0]))
  if args.maps:
    save_maps(nav, args.maps)


if __name__ == "__main__":
  main()
