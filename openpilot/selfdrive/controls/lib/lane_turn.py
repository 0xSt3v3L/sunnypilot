from openpilot.cereal import custom
from openpilot.common.constants import CV

# Lane turn desire, ported from sunnypilot (lane_turn_desire.py): below LANE_TURN_SPEED_MAX with one blinker, tell the
# model to turn, unless the blind spot on that side is occupied. Off unless LaneTurnDesire is set.

TurnDirection = custom.ModelDataSP.TurnDirection

LANE_TURN_SPEED_MAX = 19 * CV.MPH_TO_MS  # sunnypilot's default, below the 20 mph lane change minimum


def lane_turn_direction(carstate, enabled: bool):
  if not enabled or carstate.vEgo >= LANE_TURN_SPEED_MAX:
    return TurnDirection.none
  if carstate.leftBlinker and not carstate.rightBlinker and not carstate.leftBlindspot:
    return TurnDirection.turnLeft
  if carstate.rightBlinker and not carstate.leftBlinker and not carstate.rightBlindspot:
    return TurnDirection.turnRight
  return TurnDirection.none
