from opendbc.car.structs import car
from openpilot.cereal import log
from openpilot.common.realtime import DT_CTRL
from openpilot.common.test import OpenpilotTestCase
from openpilot.selfdrive.selfdrived.events import EVENTS, ET
from openpilot.selfdrive.selfdrived.standstill_alerts import (StandstillAlerts, GREEN_LIGHT_X_THRESHOLD, LEAD_DEPART_DIST_THRESHOLD,
                                                              LEAD_DEPART_MIN_SPEED, CLOSE_LEAD_DIST, TRIGGER_TIME, LEAD_ARM_TIME,
                                                              RECENT_MOVING_TIME)

EventName = log.OnroadEvent.EventName
AudibleAlert = car.CarControl.HUDControl.AudibleAlert

GO_X = GREEN_LIGHT_X_THRESHOLD + 10.
STOP_X = GREEN_LIGHT_X_THRESHOLD - 25.


def car_state(moving=False, gas=False):
  return car.CarState.new_message(standstill=not moving, vEgo=5. if moving else 0., gasPressed=gas)


def model(x_end):
  return log.ModelDataV2.new_message(position={"x": [0., x_end / 2, x_end]})


def radar(d_rel=None, v_lead=0.):
  lead = {"present": False} if d_rel is None else {"present": True, "dRel": d_rel, "vLead": v_lead}
  return log.RadarState.new_message(leadOne=lead)


class Sim:
  def __init__(self):
    self.alerts = StandstillAlerts()
    self.green_count = 0
    self.lead_count = 0

  def run(self, seconds, moving=False, gas=False, enabled=False, x_end=STOP_X, d_rel=None, v_lead=0.):
    CS, mv, rs = car_state(moving, gas), model(x_end), radar(d_rel, v_lead)
    for _ in range(int(round(seconds / DT_CTRL))):
      green, lead = self.alerts.update(CS, enabled, mv, rs)
      self.green_count += green
      self.lead_count += lead
    return self

  def stop(self, x_end=STOP_X, d_rel=None, enabled=False):
    # drive, then stop long enough for the alerts to arm
    self.run(1., moving=True)
    return self.run(RECENT_MOVING_TIME + LEAD_ARM_TIME + 0.2, x_end=x_end, d_rel=d_rel, enabled=enabled)


class TestGreenLight(OpenpilotTestCase):
  def test_alerts_once_when_model_wants_to_go(self):
    sim = Sim().stop()
    assert sim.green_count == 0
    sim.run(TRIGGER_TIME + 0.1, x_end=GO_X)
    assert sim.green_count == 1
    sim.run(5., x_end=GO_X)
    assert sim.green_count == 1

  def test_needs_trigger_time(self):
    sim = Sim().stop()
    sim.run(TRIGGER_TIME - 0.1, x_end=GO_X)
    sim.run(1.)
    assert sim.green_count == 0

  def test_not_before_recently_moving_time(self):
    sim = Sim().run(1., moving=True)
    sim.run(RECENT_MOVING_TIME - 0.5, x_end=GO_X)
    assert sim.green_count == 0

  def test_not_before_first_drive(self):
    assert Sim().run(10., x_end=GO_X).green_count == 0

  def test_not_with_lead(self):
    sim = Sim().stop(d_rel=5.)
    sim.run(2., x_end=GO_X, d_rel=5.)
    assert sim.green_count == 0

  def test_not_while_openpilot_controls_longitudinal(self):
    sim = Sim().stop(enabled=True)
    sim.run(2., x_end=GO_X, enabled=True)
    assert sim.green_count == 0

  def test_not_while_pressing_gas(self):
    sim = Sim().stop()
    sim.run(2., x_end=GO_X, gas=True)
    assert sim.green_count == 0

  def test_rearms_after_driving(self):
    sim = Sim().stop()
    sim.run(TRIGGER_TIME + 0.1, x_end=GO_X)
    sim.stop()
    sim.run(TRIGGER_TIME + 0.1, x_end=GO_X)
    assert sim.green_count == 2


class TestLeadDeparture(OpenpilotTestCase):
  def test_alerts_once_when_lead_moves_away(self):
    sim = Sim().stop(d_rel=4.)
    assert sim.lead_count == 0
    sim.run(TRIGGER_TIME + 0.1, d_rel=4. + LEAD_DEPART_DIST_THRESHOLD + 0.5, v_lead=LEAD_DEPART_MIN_SPEED + 0.5)
    assert sim.lead_count == 1
    sim.run(2., d_rel=15., v_lead=LEAD_DEPART_MIN_SPEED + 0.5)
    assert sim.lead_count == 1

  def test_lead_stopped_close_to_the_limit_still_alerts(self):
    # once armed, the lead may move past the close lead distance before the alert triggers
    d_rel = CLOSE_LEAD_DIST - 0.5
    sim = Sim().stop(d_rel=d_rel)
    sim.run(TRIGGER_TIME + 0.1, d_rel=d_rel + LEAD_DEPART_DIST_THRESHOLD + 0.5, v_lead=LEAD_DEPART_MIN_SPEED + 0.5)
    assert sim.lead_count == 1

  def test_lead_switch_without_motion_ignored(self):
    # route 000000a3: the radar lead jumped between stopped scooters, dRel grew but nothing moved
    sim = Sim().stop(d_rel=4.9)
    sim.run(0.4, d_rel=2.8)
    sim.run(1., d_rel=5.2, v_lead=0.03)
    assert sim.lead_count == 0

  def test_small_movement_ignored(self):
    sim = Sim().stop(d_rel=4.)
    sim.run(2., d_rel=4. + LEAD_DEPART_DIST_THRESHOLD - 0.3, v_lead=LEAD_DEPART_MIN_SPEED + 0.5)
    assert sim.lead_count == 0

  def test_far_lead_does_not_arm(self):
    sim = Sim().stop(d_rel=CLOSE_LEAD_DIST + 2.)
    sim.run(2., d_rel=CLOSE_LEAD_DIST + 5.)
    assert sim.lead_count == 0

  def test_lead_lost(self):
    sim = Sim().stop(d_rel=4.)
    sim.run(2.)
    assert sim.lead_count == 0

  def test_not_while_openpilot_controls_longitudinal(self):
    sim = Sim().stop(d_rel=4., enabled=True)
    sim.run(2., d_rel=8., enabled=True, v_lead=LEAD_DEPART_MIN_SPEED + 0.5)
    assert sim.lead_count == 0

  def test_lead_departure_does_not_also_alert_green(self):
    sim = Sim().stop(d_rel=4.)
    sim.run(2., d_rel=9., x_end=GO_X, v_lead=LEAD_DEPART_MIN_SPEED + 0.5)
    assert sim.lead_count == 1
    assert sim.green_count == 0


class TestStandstillAlertEvents(OpenpilotTestCase):
  def test_events_defined(self):
    for name in ("greenLight", "leadDeparted"):
      event = EventName.schema.enumerants[name]
      alert = EVENTS[event][ET.PERMANENT]
      assert alert.audible_alert == AudibleAlert.prompt, name
      assert alert.alert_text_1, name
