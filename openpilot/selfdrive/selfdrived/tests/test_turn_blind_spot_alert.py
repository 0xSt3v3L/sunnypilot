import os

from opendbc.car.structs import car
from openpilot.cereal import log
from openpilot.common.basedir import BASEDIR
from openpilot.common.realtime import DT_CTRL
from openpilot.common.test import OpenpilotTestCase
from openpilot.selfdrive.selfdrived.events import EVENTS, ET
from openpilot.selfdrive.selfdrived.turn_blind_spot_alert import TurnBlindSpotAlert, MAX_SPEED, COOLDOWN

AudibleAlert = car.CarControl.HUDControl.AudibleAlert
EventName = log.OnroadEvent.EventName
SLOW = MAX_SPEED - 2.


def cs(v=SLOW, right=False, left=False, right_bsm=False, left_bsm=False):
  return car.CarState.new_message(vEgo=v, rightBlinker=right, leftBlinker=left,
                                  rightBlindspot=right_bsm, leftBlindspot=left_bsm)


def right_turn(**kw):
  return cs(**{"right": True, "right_bsm": True, **kw})


def left_turn(**kw):
  return cs(**{"left": True, "left_bsm": True, **kw})


def count(alert, CS, seconds):
  """(left, right) alerts raised over the given time"""
  left = right = 0
  for _ in range(int(round(seconds / DT_CTRL))):
    l, r = alert.update(CS)
    left += l
    right += r
  return left, right


def read_selfdrived():
  with open(os.path.join(BASEDIR, "openpilot/selfdrive/selfdrived/selfdrived.py")) as f:
    return f.read()


class TestTurnBlindSpotAlert(OpenpilotTestCase):
  def test_alerts_once_when_the_right_blind_spot_fills(self):
    a = TurnBlindSpotAlert()
    assert count(a, right_turn(right_bsm=False), 1.) == (0, 0)
    assert count(a, right_turn(), 3.) == (0, 1)

  def test_alerts_once_when_the_left_blind_spot_fills(self):
    a = TurnBlindSpotAlert()
    assert count(a, left_turn(left_bsm=False), 1.) == (0, 0)
    assert count(a, left_turn(), 3.) == (1, 0)

  def test_needs_the_blinker_on_the_blind_spot_side_only(self):
    assert count(TurnBlindSpotAlert(), right_turn(right=False), 1.) == (0, 0)
    assert count(TurnBlindSpotAlert(), right_turn(left=True), 1.) == (0, 0)
    assert count(TurnBlindSpotAlert(), left_turn(left=False), 1.) == (0, 0)
    assert count(TurnBlindSpotAlert(), left_turn(right=True), 1.) == (0, 0)
    # blind spot on the other side
    assert count(TurnBlindSpotAlert(), cs(right=True, left_bsm=True), 1.) == (0, 0)
    assert count(TurnBlindSpotAlert(), cs(left=True, right_bsm=True), 1.) == (0, 0)

  def test_not_at_lane_change_speed(self):
    assert count(TurnBlindSpotAlert(), right_turn(v=MAX_SPEED + 0.5), 1.) == (0, 0)
    assert count(TurnBlindSpotAlert(), left_turn(v=MAX_SPEED + 0.5), 1.) == (0, 0)

  def test_flicker_within_cooldown_alerts_once(self):
    for turn, expected in ((right_turn, (0, 1)), (left_turn, (1, 0))):
      a = TurnBlindSpotAlert()
      total = (0, 0)
      for _ in range(3):
        on, off = count(a, turn(), 0.4), count(a, turn(right_bsm=False, left_bsm=False), 0.4)
        total = (total[0] + on[0] + off[0], total[1] + on[1] + off[1])
      assert total == expected

  def test_alerts_again_after_cooldown(self):
    a = TurnBlindSpotAlert()
    count(a, left_turn(), 0.5)
    count(a, left_turn(left_bsm=False), COOLDOWN + 0.5)
    assert count(a, left_turn(), 0.5) == (1, 0)

  def test_sides_are_independent(self):
    a = TurnBlindSpotAlert()
    assert count(a, right_turn(), 0.5) == (0, 1)
    # switching the blinker right away still alerts on the left
    assert count(a, left_turn(), 0.5) == (1, 0)

  def test_events_defined(self):
    for name in (EventName.rightTurnBlindSpot, EventName.leftTurnBlindSpot):
      alert = EVENTS[name][ET.PERMANENT]
      assert alert.alert_text_1 and alert.audible_alert == AudibleAlert.promptRepeat
    assert "Left" in EVENTS[EventName.leftTurnBlindSpot][ET.PERMANENT].alert_text_1
    assert "Right" in EVENTS[EventName.rightTurnBlindSpot][ET.PERMANENT].alert_text_1

  def test_selfdrived_raises_them(self):
    selfdrived = read_selfdrived()
    assert "self.turn_blind_spot_alert.update(CS)" in selfdrived
    assert "EventName.leftTurnBlindSpot" in selfdrived
    assert "EventName.rightTurnBlindSpot" in selfdrived


class TestBigModelReady(OpenpilotTestCase):
  def test_event_defined(self):
    alert = EVENTS[EventName.bigModelReady][ET.PERMANENT]
    assert alert.alert_text_1 == "Big Model Ready"
    assert alert.audible_alert == AudibleAlert.prompt

  def test_selfdrived_raises_it_only_after_a_successful_load(self):
    selfdrived = read_selfdrived()
    assert "if self.big_model_loading and not loading:" in selfdrived
    block = selfdrived.split("if self.big_model_loading and not loading:")[1].split("self.big_model_loading = loading")[0]
    assert "if big_active:" in block
    assert "self.events.add(EventName.bigModelReady)" in block


class TestLaneChangeRoadEdge(OpenpilotTestCase):
  def test_event_defined(self):
    alert = EVENTS[EventName.laneChangeRoadEdge][ET.WARNING]
    assert "Road Edge" in alert.alert_text_1
    assert alert.audible_alert == AudibleAlert.prompt

  def test_selfdrived_raises_it_instead_of_the_pre_lane_change_prompt(self):
    selfdrived = read_selfdrived()
    assert "'modelDataSP'" in selfdrived
    lane_change = selfdrived.split("# Handle lane change")[1].split("elif self.sm['modelV2'].meta.laneChangeState in")[0]
    assert "road_edge.leftDetected and direction == LaneChangeDirection.left" in lane_change
    assert "road_edge.rightDetected and direction == LaneChangeDirection.right" in lane_change
    # blind spot first, then road edge, then the stock prompt
    assert lane_change.index("EventName.laneChangeBlocked") < lane_change.index("EventName.laneChangeRoadEdge") \
           < lane_change.index("EventName.preLaneChangeLeft")
