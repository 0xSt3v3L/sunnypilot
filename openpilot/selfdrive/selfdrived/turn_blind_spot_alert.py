from openpilot.common.constants import CV
from openpilot.common.realtime import DT_CTRL

# Turn blind spot alert (Taiwan: scooters pass on the right of a car turning right, and on either side when merging).
# Alerts once when a blind spot fills while only the blinker on that side is on, below the 20 mph lane change speed
# where stock openpilot already warns about a blind spot. This car's BSM works down to about 9-10 km/h, so the slowest
# part of a turn is not covered.

MAX_SPEED = 20 * CV.MPH_TO_MS
COOLDOWN = 3.  # s; the blind spot signal flickers while a scooter passes


class _Side:
  def __init__(self):
    self.active_prev = False
    self.cooldown = 0.

  def update(self, active: bool) -> bool:
    self.cooldown = max(self.cooldown - DT_CTRL, 0.)
    alert = active and not self.active_prev and self.cooldown == 0.
    if alert:
      self.cooldown = COOLDOWN
    self.active_prev = active
    return alert


class TurnBlindSpotAlert:
  def __init__(self):
    self.left = _Side()
    self.right = _Side()

  def update(self, CS) -> tuple[bool, bool]:
    """(left, right) alerts"""
    slow = CS.vEgo < MAX_SPEED
    left = CS.leftBlinker and not CS.rightBlinker and CS.leftBlindspot and slow
    right = CS.rightBlinker and not CS.leftBlinker and CS.rightBlindspot and slow
    return self.left.update(left), self.right.update(right)
