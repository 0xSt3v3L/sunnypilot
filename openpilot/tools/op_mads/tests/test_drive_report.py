import math
from openpilot.cereal import log
from openpilot.common.test import OpenpilotTestCase
from openpilot.tools.op_mads.drive_report import (summary, mads_time, standstill_report, lane_turn_report, road_edge_report, bsm_report,
                                                  curve_report, format_report)

EventName = log.OnroadEvent.EventName
LaneChangeState = log.LaneChangeState
LaneChangeDirection = log.LaneChangeDirection
DT = 0.01
DT_MDL = 0.05


def at(t, **kwargs):
  msg = log.Event.new_message(**kwargs)
  msg.logMonoTime = int(round(t * 1e9))
  return msg.as_reader()


def ticks(t0, t1, dt=DT):
  return [t0 + i * dt for i in range(int(round((t1 - t0) / dt)))]


def ordered(msgs):
  return sorted(msgs, key=lambda m: m.logMonoTime)


def car_params(flags=0, steer_ratio=15., wheelbase=2.7):
  return at(0., carParams={"carFingerprint": "TOYOTA_COROLLA_TSS2", "brand": "toyota", "flags": flags, "alternativeExperience": 1024,
                           "steerRatio": steer_ratio, "wheelbase": wheelbase})


class TestSummary(OpenpilotTestCase):
  def test_summary(self):
    msgs = ordered([car_params(flags=3)] + [at(t, carState={"vEgo": 10.}) for t in ticks(0., 10.)] +
                   [at(t, modelV2={"big": t < 5.}) for t in ticks(0., 10., DT_MDL)])
    s = summary(msgs)
    assert s["fingerprint"] == "TOYOTA_COROLLA_TSS2"
    assert s["has_bsm"] is True and s["mads_available"] is True
    assert abs(s["duration_s"] - 10.) < 0.1
    assert abs(s["distance_km"] - 0.1) < 0.002
    assert abs(s["big_ratio"] - 0.5) < 0.02

  def test_no_bsm_flag(self):
    assert summary([car_params(flags=1)])["has_bsm"] is False


class TestMadsTime(OpenpilotTestCase):
  def test_time_split(self):
    msgs = []
    for t in ticks(0., 4.):
      msgs.append(at(t, selfdriveState={"enabled": t >= 3.}))
      msgs.append(at(t + 0.001, selfdriveStateSP={"mads": {"available": True, "active": t >= 1.}}))
      msgs.append(at(t + 0.002, carState={"vEgo": 5.}))
      msgs.append(at(t + 0.003, carControl={"latActive": t >= 1.}))
    msgs.append(at(2., onroadEvents=[{"name": EventName.controlsMismatchLateral}]))
    m = mads_time(ordered(msgs))
    assert abs(m["off_s"] - 1.) < 0.05
    assert abs(m["lat_only_s"] - 2.) < 0.05
    assert abs(m["full_s"] - 1.) < 0.05
    assert abs(m["lat_moving_s"] - 3.) < 0.05
    assert m["mismatches"] == 1


class TestStandstill(OpenpilotTestCase):
  def stop(self, alert):
    msgs = [at(0., selfdriveState={"enabled": False}), at(0., radarState={"leadOne": {"present": False}})]
    msgs += [at(t, carState={"standstill": t >= 1., "vEgo": 0. if t >= 1. else 5.}) for t in ticks(0., 5.)]
    msgs += [at(t, modelV2={"position": {"x": [0., 20., 40.]}}) for t in ticks(3., 4., DT_MDL)]
    if alert:
      msgs.append(at(4., onroadEvents=[{"name": EventName.greenLight}]))
    return standstill_report(ordered(msgs))

  def test_alert_with_context(self):
    r = self.stop(alert=True)
    assert len(r["alerts"]) == 1
    a = r["alerts"][0]
    assert a["name"] == "greenLight" and a["lead_d"] is None and abs(a["plan_x"] - 40.) < 1e-6
    assert len(r["stops"]) == 1 and r["stops"][0]["alerted"] and not r["stops"][0]["lead"]
    assert abs(r["stops"][0]["duration"] - 4.) < 0.05

  def test_stop_without_alert(self):
    r = self.stop(alert=False)
    assert r["alerts"] == [] and not r["stops"][0]["alerted"]


  def test_stop_before_first_drive_ignored(self):
    # the standstill alerts only arm after the car has moved
    msgs = [at(0., selfdriveState={"enabled": False})] + [at(t, carState={"standstill": True}) for t in ticks(0., 5.)]
    assert standstill_report(ordered(msgs))["stops"] == []


class TestLaneTurn(OpenpilotTestCase):
  def test_turn_with_curvature_change(self):
    msgs = [at(t, carState={"vEgo": 5.}) for t in ticks(0., 4.)]
    msgs += [at(t, modelDataSP={"laneTurnDirection": "turnLeft" if t >= 2. else "none"}) for t in ticks(0., 4., DT_MDL)]
    msgs += [at(t, modelV2={"action": {"desiredCurvature": 0.02 if t >= 2. else 0.}}) for t in ticks(0., 4., DT_MDL)]
    turns = lane_turn_report(ordered(msgs))
    assert len(turns) == 1
    turn = turns[0]
    assert turn["direction"] == "turnLeft" and abs(turn["v_kph"] - 18.) < 0.1
    assert abs(turn["curv_before"]) < 1e-6 and abs(turn["curv_after"] - 0.02) < 1e-6


class TestRoadEdgeAndBsm(OpenpilotTestCase):
  def test_road_edge(self):
    msgs = [at(t, modelDataSP={"roadEdge": {"leftDetected": 1. <= t < 2.}}) for t in ticks(0., 3., DT_MDL)]
    msgs += [at(t, carState={"steeringPressed": True, "steeringTorque": 50.}) for t in ticks(1.2, 1.6)]
    msgs += [at(t + 0.001, modelV2={"meta": {"laneChangeState": LaneChangeState.preLaneChange, "laneChangeDirection": LaneChangeDirection.left}})
             for t in ticks(1.3, 1.5, DT_MDL)]
    r = road_edge_report(ordered(msgs))
    assert abs(r["left_s"] - 1.) < 0.06 and r["right_s"] == 0.
    assert r["blocked"] == 1

  def test_bsm_events(self):
    pattern = [False, True, True, False, True, False]
    msgs = [at(i * DT, carState={"leftBlindspot": v}) for i, v in enumerate(pattern)]
    assert bsm_report(msgs) == {"left_events": 2, "right_events": 0}


class TestCurves(OpenpilotTestCase):
  def test_curve_with_slowdown(self):
    sr, wb, a_y = 15., 2.7, 2.
    angle_deg = math.degrees(a_y * sr * wb / 15. ** 2)
    msgs = [car_params(steer_ratio=sr, wheelbase=wb), at(0., selfdriveState={"enabled": True, "experimentalMode": True}),
            at(0., longitudinalPlan={"longitudinalPlanSource": "e2e"})]
    for t in ticks(0., 16.):
      v, angle = (20., 0.) if t < 10. else (15., angle_deg) if t < 13. else (15., 0.)
      msgs.append(at(t, carState={"vEgo": v, "steeringAngleDeg": angle}))
    msgs += [at(t, modelV2={"action": {"desiredAcceleration": -1. if 6. <= t < 10. else 0.}}) for t in ticks(0., 16., DT_MDL)]
    curves = curve_report(ordered(msgs))
    assert len(curves) == 1
    c = curves[0]
    assert abs(c["t"] - 10.) < 0.05 and abs(c["peak_ay"] - a_y) < 0.05
    assert abs(c["v_before_kph"] - 72.) < 0.1 and abs(c["v_entry_kph"] - 54.) < 0.1
    assert abs(c["model_accel_min"] + 1.) < 1e-6
    assert c["experimental"] and c["engaged"] and c["source"] == "e2e"

  def test_short_wiggle_is_not_a_curve(self):
    msgs = [car_params()] + [at(t, carState={"vEgo": 20., "steeringAngleDeg": 10. if 1. <= t < 1.5 else 0.}) for t in ticks(0., 3.)]
    assert curve_report(ordered(msgs)) == []


  def test_intersection_turn_is_not_a_curve(self):
    # a sharp turn at 15 km/h is an intersection, not a road curve
    v, sr, wb = 15. / 3.6, 15., 2.7
    angle = math.degrees(3. * sr * wb / v ** 2)
    msgs = [car_params(steer_ratio=sr, wheelbase=wb)] + [at(t, carState={"vEgo": v, "steeringAngleDeg": angle if 1. <= t < 4. else 0.})
                                                         for t in ticks(0., 5.)]
    assert curve_report(ordered(msgs)) == []


class TestFormat(OpenpilotTestCase):
  def test_format_has_sections(self):
    text = format_report([car_params()])
    for title in ("summary", "MADS", "standstill alerts", "lane turns", "road edges", "blind spot", "curves"):
      assert title in text, title
