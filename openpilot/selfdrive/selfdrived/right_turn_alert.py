from openpilot.common.constants import CV
from openpilot.common.realtime import DT_CTRL

# Right turn blind spot alert (Taiwan: scooters pass on the right of a car turning right). Alerts once when the right
# blind spot fills while only the right blinker is on, below the 20 mph lane change speed where stock openpilot already
# warns about a blind spot. This car's BSM works down to about 9-10 km/h, so the slowest part of a turn is not covered.

MAX_SPEED = 20 * CV.MPH_TO_MS
COOLDOWN = 3.  # s; the blind spot signal flickers while a scooter passes


class RightTurnAlert:
  def __init__(self):
    self.active_prev = False
    self.cooldown = 0.

  def update(self, CS) -> bool:
    active = CS.rightBlinker and not CS.leftBlinker and CS.rightBlindspot and CS.vEgo < MAX_SPEED
    self.cooldown = max(self.cooldown - DT_CTRL, 0.)
    alert = active and not self.active_prev and self.cooldown == 0.
    if alert:
      self.cooldown = COOLDOWN
    self.active_prev = active
    return alert
