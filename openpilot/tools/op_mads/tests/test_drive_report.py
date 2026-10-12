import math
import tempfile
from pathlib import Path
from openpilot.cereal import log
from openpilot.common.test import OpenpilotTestCase
from openpilot.tools.op_mads.drive_report import (summary, mads_time, standstill_report, lane_turn_report, road_edge_report, bsm_report,
                                                  curve_report, lateral_report, chestnut_report, disengagements, format_report,
                                                  route_segments, route_t0, override_report, phantom_brake_report)

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

  def test_road_edge_blocked_times(self):
    msgs = [at(t, modelDataSP={"roadEdge": {"leftDetected": True}}) for t in ticks(0., 2., DT_MDL)]
    msgs += [at(t, carState={"steeringPressed": True, "steeringTorque": 50.}) for t in ticks(0.5, 1.)]
    msgs += [at(t + 0.001, modelV2={"meta": {"laneChangeState": LaneChangeState.preLaneChange, "laneChangeDirection": LaneChangeDirection.left}})
             for t in ticks(0.6, 0.8, DT_MDL)]
    assert [round(t, 2) for t in road_edge_report(ordered(msgs))["blocked_times"]] == [0.6]

  def test_flickering_nudge_is_one_block(self):
    # steeringPressed flickers during one preLaneChange; that is one blocked lane change
    msgs = [at(t, modelDataSP={"roadEdge": {"rightDetected": True}}) for t in ticks(0., 3., DT_MDL)]
    msgs += [at(t, carState={"steeringPressed": int(t * 10) % 2 == 0, "steeringTorque": -50.}) for t in ticks(0.5, 2.)]
    msgs += [at(t + 0.001, modelV2={"meta": {"laneChangeState": LaneChangeState.preLaneChange, "laneChangeDirection": LaneChangeDirection.right}})
             for t in ticks(0.5, 2., DT_MDL)]
    assert road_edge_report(ordered(msgs))["blocked"] == 1

  def test_bsm_events(self):
    pattern = [False, True, True, False, True, False]
    msgs = [at(i * DT, carState={"leftBlindspot": v, "vEgo": 3. + i}) for i, v in enumerate(pattern)]
    r = bsm_report(msgs)
    assert r["left_events"] == 2 and r["right_events"] == 0
    assert [e["side"] for e in r["events"]] == ["left", "left"]
    assert abs(r["lowest_kph"] - 4. * 3.6) < 1e-6

  def test_right_turn_with_right_blind_spot(self):
    msgs = [at(0., carState={"vEgo": 4., "rightBlinker": True, "rightBlindspot": False}),
            at(0.1, carState={"vEgo": 4., "rightBlinker": True, "rightBlindspot": True}),
            at(0.2, carState={"vEgo": 15., "leftBlinker": True, "leftBlindspot": False}),
            at(0.3, carState={"vEgo": 15., "leftBlinker": True, "leftBlindspot": True})]
    r = bsm_report(msgs)
    assert r["right_turn_hits"] == 1
    assert [e["blinker"] for e in r["events"]] == [True, True]


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


def torque(t, desired, actual, saturated=False):
  return at(t, controlsState={"lateralControlState": {"torqueState": {"desiredLateralAccel": desired, "actualLateralAccel": actual,
                                                                      "saturated": saturated}}})


class TestLateral(OpenpilotTestCase):
  def drive(self, v=15., pressed=lambda t: False, error=lambda t: 0.2, at_limit=lambda t: False, seconds=10.):
    msgs = []
    for t in ticks(0., seconds):
      msgs.append(at(t, carControl={"latActive": True, "actuators": {"torque": -1. if at_limit(t) else -0.3}}))
      msgs.append(at(t + 0.001, carState={"vEgo": v, "steeringPressed": pressed(t)}))
      # torqueState.saturated only turns on after seconds at the limit, so it stays off through a turn
      msgs.append(torque(t + 0.002, 1. + error(t), 1.))
    return msgs

  def test_error_by_speed_bin(self):
    r = lateral_report(ordered(self.drive(v=15.)))  # 54 km/h
    b = r["bins"]["40-80"]
    assert abs(b["mean"] - 0.2) < 1e-4 and abs(b["p95"] - 0.2) < 1e-4 and abs(b["time_s"] - 10.) < 0.05
    assert r["bins"]["10-40"]["time_s"] == 0.

  def test_ignores_driver_steering_and_low_speed(self):
    assert lateral_report(ordered(self.drive(v=2.)))["bins"]["10-40"]["time_s"] == 0.
    r = lateral_report(ordered(self.drive(pressed=lambda t: t < 5.)))
    assert abs(r["bins"]["40-80"]["time_s"] - 5.) < 0.05

  def test_overrides_and_saturation(self):
    msgs = self.drive(pressed=lambda t: 2. <= t < 2.5 or 6. <= t < 6.5, at_limit=lambda t: 8. <= t < 9.)
    msgs.append(at(8.5, onroadEvents=[{"name": EventName.steerSaturated}]))
    r = lateral_report(ordered(msgs))
    assert r["overrides"] == 2 and abs(r["overrides_per_10min"] - 2 / (10. / 600.)) < 1.
    assert r["saturated_episodes"] == 1 and abs(r["saturated_s"] - 1.) < 0.05
    assert [round(x["t"], 1) for x in r["steer_saturated"]] == [8.5]

  def test_flickering_press_is_one_override(self):
    # steeringPressed flickers while the driver rests a hand on the wheel
    r = lateral_report(ordered(self.drive(pressed=lambda t: 2. <= t < 3. and int(t * 10) % 2 == 0)))
    assert r["overrides"] == 1

  def test_worst_moments_are_spread(self):
    r = lateral_report(ordered(self.drive(error=lambda t: 1.5 if 3. <= t < 3.2 else 0.9 if 8. <= t < 8.2 else 0.1)))
    assert [round(x["t"]) for x in r["worst"][:2]] == [3, 8]


class TestChestnut(OpenpilotTestCase):
  def test_chestnut_health(self):
    msgs = []
    for i, t in enumerate(ticks(0., 10., 0.1)):
      m = log.Event.new_message(chestnutState={"tempC": 40. + i % 10, "memoryTempC": 50., "powerDrawW": 40., "powerLimitW": 100.,
                                               "gpuUsagePercent": 50, "gpuClockMhz": 1100, "supplyVoltage": 13500 - i,
                                               "supplyFault": 30 <= i < 32, "pcieLtssm": 120 if i < 50 else 0})
      m.logMonoTime = int(t * 1e9)
      m.valid = i % 4 != 0
      msgs.append(m.as_reader())
    msgs += [at(t, modelV2={"big": not (5. <= t < 6.)}) for t in ticks(0., 10., DT_MDL)]
    msgs.append(at(5., onroadEvents=[{"name": EventName.bigModelFailed}]))
    r = chestnut_report(ordered(msgs))
    assert r["temp_max"] == 49. and abs(r["power_avg"] - 40.) < 1e-6 and r["power_limit"] == 100.
    assert r["supply_min_v"] < 13.5 and r["supply_faults"] == 1 and r["ltssm_changes"] == 1
    assert abs(r["valid_ratio"] - 0.75) < 0.01
    assert [round(t, 1) for t in r["fallbacks"]] == [5.] and [round(t) for t in r["big_model_failed"]] == [5]


class TestDisengagements(OpenpilotTestCase):
  def test_disengagements(self):
    msgs = []
    for t in ticks(0., 4.):
      msgs.append(at(t, selfdriveState={"enabled": 1. <= t < 2.}))
      msgs.append(at(t + 0.001, selfdriveStateSP={"mads": {"available": True, "enabled": t < 3.}}))
    assert [(round(t, 2), k) for t, k in disengagements(ordered(msgs))] == [(2., "disengage"), (3., "mads off")]


class TestFormat(OpenpilotTestCase):
  def test_format_has_sections(self):
    text = format_report([car_params()])
    for title in ("summary", "MADS", "standstill alerts", "lane turns", "road edges", "blind spot", "curves", "lateral tracking", "chestnut"):
      assert title in text, title


class TestRouteSegments(OpenpilotTestCase):
  def test_skips_missing_and_partial_segments(self):
    with tempfile.TemporaryDirectory() as d:
      for seg, name in (("0", "rlog.zst"), ("1", "rlog.zst.part"), ("2", "rlog.zst"), ("10", "rlog.zst")):
        (Path(d) / seg).mkdir()
        (Path(d) / seg / name).touch()
      (Path(d) / "notes").mkdir()
      found, missing = route_segments(d)
      assert [p.parent.name for p in found] == ["0", "2", "10"]
      assert missing == [1]


class TestOverrides(OpenpilotTestCase):
  def test_override_categories(self):
    sr, wb = 15., 2.7
    curve_angle = math.degrees(2. * sr * wb / 15. ** 2)
    msgs = [car_params(steer_ratio=sr, wheelbase=wb)]
    for t in ticks(0., 15.):
      v, angle, blinker = (5., 0., t >= 4. and t < 5.) if t < 9. else (15., 0., False)
      if 3.5 <= t < 5.:
        v = 5.
      elif 6.5 <= t < 8.:
        v, angle = 15., curve_angle
      elif 9.5 <= t < 11.:
        v = 3.
      elif t < 3.5:
        v = 15.
      pressed = any(start <= t < start + 0.3 for start in (1., 4., 7., 10., 13.))
      # desiredLateralAccel > 0 steers right while driver steeringTorque > 0 steers left (route 000000a5):
      # the driver helps the +0.5 request with negative torque and opposes it at 13 s with positive torque
      trq = 50. if 13. <= t < 13.3 else -50.
      msgs.append(at(t, carState={"vEgo": v, "steeringAngleDeg": angle, "rightBlinker": blinker,
                                  "steeringPressed": pressed, "steeringTorque": trq}))
      msgs.append(at(t + 0.001, carControl={"latActive": True}))
      msgs.append(at(t + 0.002, selfdriveState={"enabled": t >= 12.}))
      msgs.append(torque(t + 0.003, 0.5, 0.5))
    msgs += [at(t + 0.004, modelV2={"meta": {"laneChangeState": LaneChangeState.preLaneChange if 0.5 <= t < 1.5 else LaneChangeState.off}})
             for t in ticks(0., 15., DT_MDL)]
    r = override_report(ordered(msgs))
    assert [e["category"] for e in r["events"]] == ["lane change", "blinker turn", "curve", "low speed", "straight"]
    assert [e["opposing"] for e in r["events"]] == [False, False, False, False, True]
    assert r["categories"]["straight"]["count"] == 1 and abs(r["categories"]["curve"]["avg_s"] - 0.3) < 0.02
    assert r["lat_only"] == 4 and r["full"] == 1


class TestPhantomBrakes(OpenpilotTestCase):
  def test_classification(self):
    v = 15.
    msgs = [at(0., longitudinalPlan={"longitudinalPlanSource": "e2e"})]
    braking = ((2., 3.), (6., 7.), (10., 11.), (14., 15.), (18., 18.3), (21., 22.))
    for t in ticks(0., 23.):
      accel = -1.5 if any(a <= t < b for a, b in braking) else 0.
      msgs.append(at(t, carState={"vEgo": v, "brakePressed": 21. <= t < 22.}))
      msgs.append(at(t + 0.001, carControl={"actuators": {"accel": accel}, "longActive": True}))
      msgs.append(at(t + 0.002, selfdriveState={"enabled": True, "experimentalMode": True}))
    for t in ticks(0., 23., DT_MDL):
      msgs.append(at(t, radarState={"leadOne": {"present": 1.5 <= t < 3., "dRel": 15.}}))
      msgs.append(at(t + 0.001, modelV2={"position": {"x": [0., 2., 5.] if 9.5 <= t < 11. else [0., 50., 150.]},
                                         "orientationRate": {"z": [0., 0.2, 0.2] if 5.5 <= t < 7. else [0., 0., 0.]}}))
    r = phantom_brake_report(ordered(msgs))
    assert [e["kind"] for e in r] == ["lead", "curve", "model stop", "unexplained"]
    assert abs(r[3]["t"] - 14.) < 0.05 and abs(r[3]["accel_min"] + 1.5) < 1e-6 and r[3]["source"] == "e2e"


  def test_planner_braking_for_a_far_closing_lead_is_lead(self):
    msgs = [at(0., longitudinalPlan={"longitudinalPlanSource": "lead0"}), at(0., radarState={"leadOne": {"present": True, "dRel": 54.}}),
            at(0., modelV2={"position": {"x": [0., 50., 150.]}})]
    for t in ticks(0., 2.):
      msgs.append(at(t, carState={"vEgo": 18.6}))
      msgs.append(at(t + 0.001, carControl={"actuators": {"accel": -2.}}))
      msgs.append(at(t + 0.002, selfdriveState={"enabled": True}))
    assert [e["kind"] for e in phantom_brake_report(ordered(msgs))] == ["lead"]


class TestRouteT0(OpenpilotTestCase):
  def test_first_message_of_the_first_segment(self):
    import zstandard
    with tempfile.TemporaryDirectory() as d:
      for seg, t in (("0", 5.), ("1", 65.)):
        (Path(d) / seg).mkdir()
        data = b"".join(log.Event.new_message(logMonoTime=int((t + i) * 1e9), carState={}).to_bytes() for i in range(3))
        (Path(d) / seg / "rlog.zst").write_bytes(zstandard.ZstdCompressor().compress(data))
      assert abs(route_t0(d) - 5.) < 1e-6
