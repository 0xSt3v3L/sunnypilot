from opendbc.car.structs import car
from openpilot.cereal import log
from openpilot.common.realtime import DT_CTRL
from openpilot.common.test import OpenpilotTestCase
from openpilot.selfdrive.selfdrived.events import EVENTS, ET
from openpilot.selfdrive.selfdrived.right_turn_alert import RightTurnAlert, MAX_SPEED, COOLDOWN

EventName = log.OnroadEvent.EventName
SLOW = MAX_SPEED - 2.


def cs(v=SLOW, right=True, left=False, bsm=True):
  return car.CarState.new_message(vEgo=v, rightBlinker=right, leftBlinker=left, rightBlindspot=bsm)


def count(alert, CS, seconds):
  return sum(alert.update(CS) for _ in range(int(round(seconds / DT_CTRL))))


class TestRightTurnAlert(OpenpilotTestCase):
  def test_alerts_once_when_the_right_blind_spot_fills(self):
    a = RightTurnAlert()
    assert count(a, cs(bsm=False), 1.) == 0
    assert count(a, cs(), 3.) == 1

  def test_needs_right_blinker_only(self):
    assert count(RightTurnAlert(), cs(right=False), 1.) == 0
    assert count(RightTurnAlert(), cs(left=True), 1.) == 0

  def test_not_at_lane_change_speed(self):
    assert count(RightTurnAlert(), cs(v=MAX_SPEED + 0.5), 1.) == 0

  def test_flicker_within_cooldown_alerts_once(self):
    a = RightTurnAlert()
    total = 0
    for _ in range(3):
      total += count(a, cs(), 0.4) + count(a, cs(bsm=False), 0.4)
    assert total == 1

  def test_alerts_again_after_cooldown(self):
    a = RightTurnAlert()
    total = count(a, cs(), 0.5) + count(a, cs(bsm=False), COOLDOWN + 0.5) + count(a, cs(), 0.5)
    assert total == 2

  def test_event_defined(self):
    alert = EVENTS[EventName.rightTurnBlindSpot][ET.PERMANENT]
    assert alert.alert_text_1 and alert.audible_alert == car.CarControl.HUDControl.AudibleAlert.promptRepeat

  def test_selfdrived_raises_it(self):
    import os
    from openpilot.common.basedir import BASEDIR
    with open(os.path.join(BASEDIR, "openpilot/selfdrive/selfdrived/selfdrived.py")) as f:
      selfdrived = f.read()
    assert "self.right_turn_alert.update(CS)" in selfdrived
    assert "EventName.rightTurnBlindSpot" in selfdrived
