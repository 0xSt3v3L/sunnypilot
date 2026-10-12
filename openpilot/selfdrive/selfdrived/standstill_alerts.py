from openpilot.common.realtime import DT_CTRL

# Green light and lead departure alerts, ported from sunnypilot (e2e_alerts_helper.py) and always enabled.
# Both only alert while openpilot does not control longitudinal and the car has been stopped for a while,
# at most once per stop. Deviations: once armed, the lead departure no longer needs the lead within
# CLOSE_LEAD_DIST, otherwise a lead stopped just inside it could move away without an alert; and the lead must
# actually move (LEAD_DEPART_MIN_SPEED), since the radar lead jumps between stopped scooters at Taiwan stop lines.

GREEN_LIGHT_X_THRESHOLD = 30.  # m, the model plan reaching this far means the model wants to go
LEAD_DEPART_DIST_THRESHOLD = 1.  # m the lead moved away from its closest distance
LEAD_DEPART_MIN_SPEED = 0.5  # m/s of lead speed; a lead switch changes dRel without motion
CLOSE_LEAD_DIST = 8.  # m, only a lead this close when stopping arms the lead departure alert
TRIGGER_TIME = 0.3
LEAD_ARM_TIME = 1.
RECENT_MOVING_TIME = 2.


class StandstillAlerts:
  def __init__(self):
    self.moved = False
    self.stopped_time = 0.
    self.allowed_prev = False

    self.green_armed = False
    self.green_consumed = False
    self.green_timer = 0.

    self.lead_confirmed = False
    self.lead_arm_timer = 0.
    self.lead_armed = False
    self.lead_consumed = False
    self.lead_min_dist = 0.
    self.lead_timer = 0.

  def update(self, CS, selfdrive_enabled: bool, model_v2, radar_state) -> tuple[bool, bool]:
    moving = not CS.standstill and CS.vEgo > 0.1
    if moving:
      self.moved = True
      self.stopped_time = 0.
    else:
      self.stopped_time += DT_CTRL
    allowed = self.moved and self.stopped_time >= RECENT_MOVING_TIME and not CS.gasPressed and not selfdrive_enabled

    lead = radar_state.leadOne
    has_lead = lead.present
    green = self._update_green(allowed and not has_lead, model_v2)
    lead_departed = self._update_lead(allowed, has_lead, lead.dRel, lead.vLead)
    self.allowed_prev = allowed
    return green, lead_departed

  def _update_green(self, allowed: bool, model_v2) -> bool:
    if not allowed:
      self.green_armed = self.green_consumed = False
      self.green_timer = 0.
      return False
    if not self.green_armed:
      self.green_armed = True
      return False
    if self.green_consumed:
      return False

    x = model_v2.position.x
    self.green_timer = self.green_timer + DT_CTRL if len(x) and x[-1] > GREEN_LIGHT_X_THRESHOLD else 0.
    if self.green_timer > TRIGGER_TIME:
      self.green_consumed = True
      return True
    return False

  def _update_lead(self, allowed: bool, has_lead: bool, d_rel: float, v_lead: float) -> bool:
    close_lead = has_lead and d_rel < CLOSE_LEAD_DIST
    # the lead must already be close when the alert becomes allowed, so a car cutting in later does not count
    if not allowed:
      self.lead_confirmed = False
    elif not self.allowed_prev and close_lead:
      self.lead_confirmed = True

    if not allowed or not has_lead:
      self.lead_armed = self.lead_consumed = False
      self.lead_arm_timer = self.lead_timer = 0.
      return False

    if not self.lead_armed:
      self.lead_arm_timer = self.lead_arm_timer + DT_CTRL if self.lead_confirmed and close_lead else 0.
      if self.lead_arm_timer >= LEAD_ARM_TIME:
        self.lead_armed = True
        self.lead_min_dist = d_rel
      return False
    if self.lead_consumed:
      return False

    self.lead_min_dist = min(self.lead_min_dist, d_rel)
    moving_away = d_rel - self.lead_min_dist > LEAD_DEPART_DIST_THRESHOLD and v_lead >= LEAD_DEPART_MIN_SPEED
    self.lead_timer = self.lead_timer + DT_CTRL if moving_away else 0.
    if self.lead_timer > TRIGGER_TIME:
      self.lead_consumed = True
      return True
    return False
