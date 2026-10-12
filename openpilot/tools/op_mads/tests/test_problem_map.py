import json
from openpilot.cereal import log
from openpilot.common.test import OpenpilotTestCase
from openpilot.tools.op_mads.problem_map import gps_track, position_at, problem_points, render_html, KINDS

EventName = log.OnroadEvent.EventName


def at(t, **kwargs):
  msg = log.Event.new_message(**kwargs)
  msg.logMonoTime = int(round(t * 1e9))
  return msg.as_reader()


def gps(t, lat, lon, fix=True):
  return at(t, gpsLocationExternal={"latitude": lat, "longitude": lon, "hasFix": fix})


class TestProblemMap(OpenpilotTestCase):
  def test_track_keeps_fixes_only(self):
    track = gps_track([gps(0., 25.0, 121.5), gps(1., 0., 0., fix=False), gps(2., 25.1, 121.6)])
    assert track == [(0., 25.0, 121.5), (2., 25.1, 121.6)]

  def test_position_at_nearest_within_gap(self):
    track = [(0., 25.0, 121.5), (10., 25.1, 121.6)]
    assert position_at(track, 1.) == (25.0, 121.5)
    assert position_at(track, 9.5) == (25.1, 121.6)
    assert position_at(track, 5.) is None

  def test_points_for_problems(self):
    msgs = [gps(t, 25.0 + t * 1e-4, 121.5) for t in range(11)]
    msgs += [at(t + 0.01, selfdriveState={"enabled": t < 4.}) for t in [x * 0.01 for x in range(1000)]]
    msgs.append(at(7., onroadEvents=[{"name": EventName.steerSaturated}]))
    msgs = sorted(msgs, key=lambda m: m.logMonoTime)
    points = problem_points(msgs, "route_a")
    kinds = sorted(p["kind"] for p in points)
    assert kinds == ["disengage", "steer saturated"]
    p = next(p for p in points if p["kind"] == "disengage")
    assert p["route"] == "route_a" and abs(p["lat"] - 25.0004) < 2e-4 and p["seg"] == 0

  def test_phantom_brake_and_route_start(self):
    msgs = [gps(t, 25.0 + t * 1e-4, 121.5) for t in range(10, 21)]
    for t in [10. + i * 0.01 for i in range(1000)]:
      msgs.append(at(t, carState={"vEgo": 15.}))
      msgs.append(at(t + 0.001, carControl={"actuators": {"accel": -1.5 if 14. <= t < 15. else 0.}}))
      msgs.append(at(t + 0.002, selfdriveState={"enabled": True}))
    msgs = sorted(msgs, key=lambda m: m.logMonoTime)
    points = problem_points(msgs, "route_b", t0=0.)
    p = next(p for p in points if p["kind"] == "phantom brake")
    assert abs(p["t"] - 14.) < 0.05 and p["seg"] == 0 and "-1.5" in p["detail"]

  def test_render_html(self):
    html = render_html([{"lat": 25.0, "lon": 121.5, "kind": "disengage", "route": "r", "seg": 0, "t": 4., "detail": "x"}])
    assert "leaflet" in html and "L.map" in html
    data = json.loads(html.split("const POINTS = ", 1)[1].split(";\n", 1)[0])
    assert data[0]["kind"] == "disengage"
    for kind in KINDS:
      assert kind in html

  def test_tiles_work_from_a_local_file(self):
    # the OSM tile servers require a Referer, which a page opened from file:// does not send
    html = render_html([])
    assert "basemaps.cartocdn.com" in html and "&copy; CARTO" in html and "OpenStreetMap contributors" in html
    assert "tile.openstreetmap.org" not in html

  def test_driver_overrides_hidden_by_default(self):
    html = render_html([])
    hidden = json.loads(html.split("const HIDDEN = ", 1)[1].split(";\n", 1)[0])
    assert hidden == ["driver override"]
