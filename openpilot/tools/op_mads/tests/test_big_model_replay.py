import numpy as np
from opendbc.car.structs import car
from openpilot.cereal import log
from openpilot.common.test import OpenpilotTestCase
from openpilot.selfdrive.controls.lib.desire_helper import DesireHelper
from openpilot.selfdrive.controls.lib.lane_turn import TurnDirection
from openpilot.selfdrive.modeld.constants import ModelConstants
from openpilot.system.camerad.cameras.nv12_info import get_nv12_info
from openpilot.tools.op_mads.big_model_replay import (MODEL_PKL, FrameSource, compare_rows, desire_from_state,
                                                      desire_vector, dump, extra_lat_action_t, frame_context,
                                                      nv12_buffer, onset_time, parse_extras, parse_segments,
                                                      raw_lat_request, runs_for, turn_events)


def car_state(v=10., left=False, right=False, pressed=False, torque=0., left_bsm=False):
  return car.CarState.new_message(vEgo=v, leftBlinker=left, rightBlinker=right, steeringPressed=pressed,
                                  steeringTorque=torque, leftBlindspot=left_bsm)


class TestBigModelReplay(OpenpilotTestCase):
  def test_desire_from_state_matches_desire_helper(self):
    dh = DesireHelper()
    steps = [
      (car_state(v=20.), True),                                    # no blinker
      (car_state(v=20., left=True), True),                         # pre lane change
      (car_state(v=20., left=True, pressed=True, torque=1.), True),  # lane change starting
      (car_state(v=20., left=True), True),
      (car_state(v=5., right=True), True),                         # lane turn right
      (car_state(v=5., left=True, left_bsm=True), True),           # lane turn blocked by the blind spot
      (car_state(v=5., left=True), True),                          # lane turn left
      (car_state(v=5., left=True), False),                         # lateral inactive
    ]
    seen = set()
    for cs, lat_active in steps:
      dh.update(cs, lat_active, 0.5, lane_turn_enabled=True)
      assert desire_from_state(dh.lane_turn_direction, dh.lane_change_state, dh.lane_change_direction) == dh.desire
      seen.add(dh.desire)
    assert {log.Desire.none, log.Desire.laneChangeLeft, log.Desire.turnLeft, log.Desire.turnRight} <= seen

  def test_desire_from_state_turn_wins(self):
    assert desire_from_state(TurnDirection.turnRight, log.LaneChangeState.laneChangeStarting,
                             log.LaneChangeDirection.left) == log.Desire.turnRight
    assert desire_from_state(TurnDirection.none, log.LaneChangeState.preLaneChange,
                             log.LaneChangeDirection.left) == log.Desire.none

  def test_desire_from_state_logged_enums(self):
    sp = log.Event.new_message(modelDataSP={"laneTurnDirection": "turnRight"}).as_reader().modelDataSP
    meta = log.Event.new_message(modelV2={"meta": {"laneChangeState": "laneChangeStarting",
                                                    "laneChangeDirection": "left"}}).as_reader().modelV2.meta
    assert desire_from_state(sp.laneTurnDirection, meta.laneChangeState, meta.laneChangeDirection) == log.Desire.turnRight
    sp = log.Event.new_message(modelDataSP={"laneTurnDirection": "none"}).as_reader().modelDataSP
    assert desire_from_state(sp.laneTurnDirection, meta.laneChangeState, meta.laneChangeDirection) == log.Desire.laneChangeLeft

  def test_desire_vector(self):
    v = desire_vector(log.Desire.turnLeft)
    assert v.shape == (ModelConstants.DESIRE_LEN,) and v.dtype == np.float32
    assert v[log.Desire.turnLeft] == 1. and v.sum() == 1.
    assert desire_vector(log.Desire.none)[0] == 1.  # modeld sets the none slot and zeroes it before the pulse

  def test_nv12_buffer_layout(self):
    w, h = 4, 2
    stride, y_height, uv_height, _ = get_nv12_info(w, h)
    img = np.arange(w * h * 3 // 2, dtype=np.uint8).reshape(h * 3 // 2, w)
    buf = nv12_buffer(img, w, h)
    assert buf.size == stride * (y_height + uv_height)
    planes = buf.reshape(y_height + uv_height, stride)
    np.testing.assert_array_equal(planes[:h, :w], img[:h])
    np.testing.assert_array_equal(planes[y_height:y_height + h // 2, :w], img[h:])
    assert planes[:h, w:].sum() == 0 and planes[h:y_height].sum() == 0

  def test_frame_source(self):
    src = FrameSource(iter(["f0", "f1", "f2", "f3"]), {100: 0, 102: 2, 103: 3})
    assert src.get(102) == "f2"
    assert src.get(102) == "f2"
    assert src.get(101) is None  # not encoded
    assert src.get(100) is None  # already passed
    assert src.get(103) == "f3"
    assert src.get(104) is None

  def test_turn_events(self):
    series = [(0., "none"), (1., "turnLeft"), (2., "turnLeft"), (3., "none"), (5., "turnRight"), (6., "turnRight")]
    assert turn_events(series) == [(1., 3., "turnLeft"), (5., 6., "turnRight")]
    assert turn_events([]) == []

  def test_turn_events_take_direction_values(self):
    series = [(0., TurnDirection.none), (1., TurnDirection.turnRight), (2., TurnDirection.none), (4., TurnDirection.none)]
    assert turn_events(series) == [(1., 2., "turnRight")]

  def test_turn_events_merge_blinker_dropouts(self):
    # the blinker signal drops out for up to about a second while waiting to turn: still one turn
    series = [(0., "turnRight"), (1., "none"), (2., "turnRight"), (3., "none"), (6., "turnRight"), (7., "none")]
    assert turn_events(series) == [(0., 3., "turnRight"), (6., 7., "turnRight")]
    assert turn_events([(0., "turnRight"), (1., "turnLeft"), (2., "none")]) == [(0., 1., "turnRight"), (1., 2., "turnLeft")]

  def test_onset_time(self):
    t = np.arange(0., 5., 0.5)
    c = np.array([0., 0., -.02, -.05, -.08, -.1, -.1, -.05, 0., 0.])
    assert onset_time(t, c, start=0., end=4.5, threshold=.05) == 1.5
    assert onset_time(t, c, start=2., end=4.5, threshold=.05) == 2.
    assert onset_time(t, c, start=0., end=4.5, threshold=.2) is None

  def test_extra_lat_action_t(self):
    assert extra_lat_action_t(car_state(v=5., right=True), .3) == .3
    assert extra_lat_action_t(car_state(v=5.), .3) == 0.
    assert extra_lat_action_t(car_state(v=20., right=True), .3) == 0.

  def test_parse_extras(self):
    assert parse_extras("0.3,0.6") == [.3, .6]

  def test_parse_segments(self):
    assert parse_segments(None, [0, 1, 2]) == [0, 1, 2]
    assert parse_segments("1-2", [0, 1, 2]) == [1, 2]
    assert parse_segments("2", [0, 1, 2]) == [2]
    with self.assertRaises(SystemExit):
      parse_segments("3-4", [0, 1, 2])

  def test_compare_rows(self):
    t = np.arange(10.)
    logged = np.array([[0., 1., 0., 1., 0., 1., 0., 1., 0., 1.]]).T
    rows = compare_rows(["x"], t, logged, logged + 0.1, skip=2.)
    (name, corr, rmse, std), = rows
    assert name == "x" and abs(corr - 1.) < 1e-9 and abs(rmse - 0.1) < 1e-9 and abs(std - 0.5) < 1e-9

  def test_runs_for_compares_models_with_the_logged_desire(self):
    loaded = []
    def load(pkl):
      loaded.append(pkl)
      return pkl
    runs = runs_for("logged", load, MODEL_PKL, None, [])
    assert [(r.name, r.model, r.lane_turn) for r in runs] == [("replay", MODEL_PKL, None)]
    loaded.clear()
    runs = runs_for("logged", load, "a.pkl", "b.pkl", [])
    assert [(r.name, r.model, r.lane_turn) for r in runs] == [("a.pkl", "a.pkl", None), ("b.pkl", "b.pkl", None)]
    assert loaded == ["a.pkl", "b.pkl"]
    with self.assertRaises(SystemExit):
      runs_for("ab", load, "a.pkl", "b.pkl", [])

  def test_runs_for_other_modes_use_the_chosen_model(self):
    runs = runs_for("ab", lambda pkl: pkl, "a.pkl", None, [])
    assert [(r.name, r.model, r.lane_turn) for r in runs] == [("lane turn off", "a.pkl", False), ("lane turn on", "a.pkl", True)]
    runs = runs_for("action_t", lambda pkl: pkl, "a.pkl", None, [.3])
    assert [(r.name, r.action_t_extra) for r in runs] == [("baseline", 0.), ("action_t +0.3s", .3)]

  def test_raw_lat_request(self):
    assert raw_lat_request({"action": np.array([[0.02, -1.5]], dtype=np.float32)}) == np.float32(0.02)
    assert np.isnan(raw_lat_request({"plan": np.zeros((1, 33, 15))}))

  def test_frame_context(self):
    cs = car.CarState.new_message(vEgo=3., aEgo=-1., steeringAngleDeg=12., leftBlinker=True)
    cc = car.CarControl.new_message(latActive=True, longActive=False)
    rs = log.RadarState.new_message(leadOne={"present": True, "dRel": 20., "vLead": 4.}).as_reader()
    ctx = frame_context(cs, cc, rs)
    assert ctx == {"v_ego": 3., "a_ego": -1., "steering_angle": 12., "left_blinker": 1., "right_blinker": 0.,
                   "lat_active": 1., "long_active": 0., "lead_present": 1., "lead_d": 20., "lead_v": 4.}
    ctx = frame_context(cs, None, None)
    assert ctx["lat_active"] == 0. and ctx["lead_present"] == 0. and np.isnan(ctx["lead_d"])

  def test_dump_round_trip(self):
    import tempfile
    from pathlib import Path
    from types import SimpleNamespace
    runs = [SimpleNamespace(name="a.pkl", rows=[{"x": 1., "lat_accel_request": .1}, {"x": 2., "lat_accel_request": .2}])]
    with tempfile.TemporaryDirectory() as d:
      path = Path(d) / "out.npz"
      dump(path, 100., [100.5, 101.], [{"x": 1.5}, {"x": 2.5}], runs, [{"v_ego": 1.}, {"v_ego": 2.}])
      z = np.load(path)
      np.testing.assert_allclose(z["t"], [.5, 1.])
      np.testing.assert_allclose(z["logged/x"], [1.5, 2.5])
      np.testing.assert_allclose(z["run0/x"], [1., 2.])
      np.testing.assert_allclose(z["run0/lat_accel_request"], [.1, .2])
      np.testing.assert_allclose(z["ctx/v_ego"], [1., 2.])
      assert list(z["run_names"]) == ["a.pkl"]
