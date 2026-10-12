#!/usr/bin/env python3
"""Map recurring problems across routes: disengagements, saturation, driver overrides, blocked lane changes, missed
standstill alerts, hard model braking in curves and suspected phantom braking.

usage: problem_map.py <logs dir with <route>/<segment>/rlog.zst> [-o out.html]
The map holds driving positions: it is written next to the logs, never into the repo.
"""
import argparse
import bisect
import json
from collections import Counter
from pathlib import Path

from openpilot.tools.op_mads.drive_report import (SERVICES, load_route, route_t0, disengagements, lateral_report, road_edge_report,
                                                  standstill_report, curve_report, override_report, phantom_brake_report)

KINDS = {
  "disengage": "#d62728",
  "mads off": "#ff7f0e",
  "steer saturated": "#9467bd",
  "driver override": "#8c564b",
  "edge blocked": "#1f77b4",
  "missed alert": "#e377c2",
  "curve hard brake": "#17becf",
  "phantom brake": "#bcbd22",
}
HIDDEN = ("driver override",)  # frequent; off until picked in the layer control
GPS_SERVICES = ("gpsLocationExternal", "gpsLocation")
MAX_GPS_GAP = 2.  # s between a problem and the nearest GPS fix
HARD_BRAKE = -1.5  # m/s^2 of model desired acceleration before a curve
SEGMENT = 60.  # s


def gps_track(msgs) -> list[tuple[float, float, float]]:
  track = []
  for m in msgs:
    if m.which() in GPS_SERVICES:
      g = getattr(m, m.which())
      if g.hasFix:
        track.append((m.logMonoTime * 1e-9, g.latitude, g.longitude))
  return track


def position_at(track, t: float, max_gap: float = MAX_GPS_GAP):
  i = bisect.bisect_left([p[0] for p in track], t)
  near = [p for p in track[max(i - 1, 0):i + 1] if abs(p[0] - t) <= max_gap]
  if not near:
    return None
  _, lat, lon = min(near, key=lambda p: abs(p[0] - t))
  return lat, lon


def problem_points(msgs, route: str, t0: float | None = None) -> list[dict]:
  """t0 is the route start (route_t0), so times match drive_report.py and frames.py."""
  if not msgs:
    return []
  t0 = msgs[0].logMonoTime * 1e-9 if t0 is None else t0
  items = [(t, kind, "") for t, kind in disengagements(msgs)]
  lat = lateral_report(msgs)
  items += [(x["t"], "steer saturated", f"desired {x['desired']:+.2f} m/s^2" if x["desired"] is not None else "") for x in lat["steer_saturated"]]
  items += [(e["t"], "driver override", f"{e['category']}{', opposing' if e['opposing'] else ''}") for e in override_report(msgs)["events"]]
  items += [(t, "edge blocked", "") for t in road_edge_report(msgs)["blocked_times"]]
  items += [(s["t"], "missed alert", f"stop {s['duration']:.0f}s, {'lead' if s['lead'] else 'no lead'}")
            for s in standstill_report(msgs)["stops"] if not s["alerted"]]
  items += [(c["t"], "curve hard brake", f"model {c['model_accel_min']:+.2f} m/s^2, peak {c['peak_ay']:.1f} m/s^2 lateral")
            for c in curve_report(msgs) if c["model_accel_min"] is not None and c["model_accel_min"] <= HARD_BRAKE]
  items += [(x["t"], "phantom brake", f"{x['accel_min']:+.1f} m/s^2 at {x['v_kph']:.0f} km/h, source {x['source']}")
            for x in phantom_brake_report(msgs) if x["kind"] == "unexplained"]

  track = gps_track(msgs)
  points = []
  for t, kind, detail in sorted(items):
    pos = position_at(track, t)
    if pos is not None:
      rel = t - t0
      points.append({"lat": pos[0], "lon": pos[1], "kind": kind, "route": route, "seg": int(rel // SEGMENT), "t": rel, "detail": detail})
  return points


def _js(value) -> str:
  return json.dumps(value).replace("<", "\\u003c")


def render_html(points: list[dict]) -> str:
  return f"""<!doctype html>
<html lang="en">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>Problem Map</title>
<link rel="stylesheet" href="https://cdnjs.cloudflare.com/ajax/libs/leaflet/1.9.4/leaflet.min.css">
<script src="https://cdnjs.cloudflare.com/ajax/libs/leaflet/1.9.4/leaflet.min.js"></script>
<style>html, body, #map {{ height: 100%; margin: 0; }}</style>
</head>
<body>
<div id="map"></div>
<script>
const POINTS = {_js(points)};
const KINDS = {_js(KINDS)};
const HIDDEN = {_js(list(HIDDEN))};
const map = L.map("map");
// Esri tiles: the OSM tile servers block pages opened from file://, which send no Referer, and CARTO needs an API key
L.tileLayer("https://server.arcgisonline.com/ArcGIS/rest/services/World_Street_Map/MapServer/tile/{{z}}/{{y}}/{{x}}",
            {{maxZoom: 19, attribution: "Tiles &copy; Esri"}}).addTo(map);
const esc = s => String(s).replace(/[&<>"]/g, c => ({{"&": "&amp;", "<": "&lt;", ">": "&gt;", '"': "&quot;"}})[c]);
const layers = {{}};
for (const [kind, color] of Object.entries(KINDS)) {{
  const pts = POINTS.filter(p => p.kind === kind);
  const group = L.layerGroup(pts.map(p => {{
    const mmss = `${{Math.floor(p.t / 60)}}:${{String(Math.floor(p.t % 60)).padStart(2, "0")}}`;
    return L.circleMarker([p.lat, p.lon], {{radius: 7, color, fillColor: color, fillOpacity: 0.8}})
      .bindPopup(`<b>${{esc(kind)}}</b><br>${{esc(p.route)}} at ${{mmss}} (segment ${{p.seg}})${{p.detail ? "<br>" + esc(p.detail) : ""}}`);
  }}));
  if (!HIDDEN.includes(kind)) group.addTo(map);
  layers[`<span style="color:${{color}}">&#9679;</span> ${{esc(kind)}} (${{pts.length}})`] = group;
}}
L.control.layers(null, layers, {{collapsed: false}}).addTo(map);
if (POINTS.length) map.fitBounds(POINTS.map(p => [p.lat, p.lon]), {{padding: [30, 30]}});
else map.setView([23.7, 121.0], 7);
</script>
</body>
</html>
"""


def route_dirs(logs_dir: str) -> list[Path]:
  return sorted(d for d in Path(logs_dir).iterdir()
                if d.is_dir() and any(s.name.isdigit() and (s / "rlog.zst").is_file() for s in d.iterdir()))


def main() -> None:
  parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
  parser.add_argument("logs_dir")
  parser.add_argument("-o", "--output", help="default: <logs dir>/problem_map.html")
  args = parser.parse_args()

  points = []
  for route in route_dirs(args.logs_dir):
    route_points = problem_points(load_route(str(route), SERVICES | set(GPS_SERVICES)), route.name, route_t0(str(route)))
    print(f"{route.name}: {len(route_points)} problems with a GPS position")
    points += route_points
  out = Path(args.output or Path(args.logs_dir) / "problem_map.html")
  out.write_text(render_html(points))
  for kind, n in Counter(p["kind"] for p in points).most_common():
    print(f"  {kind}: {n}")
  print(out)


if __name__ == "__main__":
  main()
