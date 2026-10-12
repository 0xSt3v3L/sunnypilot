import os
from opendbc.car.structs import car
from openpilot.cereal import log, custom
from openpilot.common.basedir import BASEDIR
from openpilot.common.params import Params
from openpilot.common.test import OpenpilotTestCase
from openpilot.selfdrive.controls.lib.desire_helper import DesireHelper, LANE_CHANGE_SPEED_MIN
from openpilot.selfdrive.controls.lib.lane_turn import lane_turn_direction, LANE_TURN_SPEED_MAX

TurnDirection = custom.ModelDataSP.TurnDirection
SLOW = LANE_TURN_SPEED_MAX - 1.


def cs(v_ego=SLOW, left=False, right=False, left_bsm=False, right_bsm=False):
  return car.CarState.new_message(vEgo=v_ego, leftBlinker=left, rightBlinker=right, leftBlindspot=left_bsm, rightBlindspot=right_bsm)


class TestLaneTurnDirection(OpenpilotTestCase):
  def test_turn_left_and_right(self):
    assert lane_turn_direction(cs(left=True), True) == TurnDirection.turnLeft
    assert lane_turn_direction(cs(right=True), True) == TurnDirection.turnRight

  def test_none_without_one_blinker(self):
    assert lane_turn_direction(cs(), True) == TurnDirection.none
    assert lane_turn_direction(cs(left=True, right=True), True) == TurnDirection.none

  def test_none_at_speed(self):
    assert lane_turn_direction(cs(v_ego=LANE_TURN_SPEED_MAX + 0.1, left=True), True) == TurnDirection.none

  def test_blind_spot_blocks_that_side(self):
    assert lane_turn_direction(cs(left=True, left_bsm=True), True) == TurnDirection.none
    assert lane_turn_direction(cs(left=True, right_bsm=True), True) == TurnDirection.turnLeft

  def test_disabled(self):
    assert lane_turn_direction(cs(left=True), False) == TurnDirection.none

  def test_below_lane_change_speed(self):
    # a lane turn never competes with a lane change
    assert LANE_TURN_SPEED_MAX < LANE_CHANGE_SPEED_MIN


class TestDesireHelperLaneTurn(OpenpilotTestCase):
  def test_turn_desire_even_without_lateral(self):
    DH = DesireHelper()
    DH.update(cs(left=True), False, 0., lane_turn_enabled=True)
    assert DH.lane_turn_direction == TurnDirection.turnLeft
    assert DH.desire == log.Desire.turnLeft

  def test_right_turn_desire(self):
    DH = DesireHelper()
    DH.update(cs(right=True), True, 0., lane_turn_enabled=True)
    assert DH.desire == log.Desire.turnRight

  def test_disabled_by_default(self):
    DH = DesireHelper()
    DH.update(cs(left=True), True, 0.)
    assert DH.lane_turn_direction == TurnDirection.none
    assert DH.desire == log.Desire.none

  def test_lane_change_unaffected_at_speed(self):
    DH = DesireHelper()
    fast = LANE_CHANGE_SPEED_MIN + 2.
    DH.update(cs(v_ego=fast, left=True), True, 0., lane_turn_enabled=True)
    nudge = car.CarState.new_message(vEgo=fast, leftBlinker=True, steeringPressed=True, steeringTorque=1.)
    DH.update(nudge, True, 0., lane_turn_enabled=True)
    assert DH.desire == log.Desire.laneChangeLeft


class TestLaneTurnPlumbing(OpenpilotTestCase):
  def test_param_defaults_off(self):
    params = Params()
    assert not params.get_bool("LaneTurnDesire")
    params.put_bool("LaneTurnDesire", True, block=True)
    assert params.get_bool("LaneTurnDesire")

  def test_model_data_sp_field(self):
    msg = custom.ModelDataSP.new_message(laneTurnDirection=TurnDirection.turnRight)
    assert msg.laneTurnDirection == TurnDirection.turnRight

  def test_modeld_reads_param_and_publishes(self):
    # modeld cannot be imported without its model runtime, so check its source
    with open(os.path.join(BASEDIR, "openpilot/selfdrive/modeld/modeld.py")) as f:
      modeld = f.read()
    assert 'params.get_bool("LaneTurnDesire")' in modeld
    assert "lane_turn_enabled=lane_turn_enabled" in modeld
    assert "model_data_sp_send.modelDataSP.laneTurnDirection = DH.lane_turn_direction" in modeld

  def test_toggle_in_mici_settings(self):
    with open(os.path.join(BASEDIR, "openpilot/selfdrive/ui/mici/layouts/settings/toggles.py")) as f:
      toggles = f.read()
    assert '"LaneTurnDesire"' in toggles
    assert '("LaneTurnDesire", lane_turn_toggle)' in toggles
