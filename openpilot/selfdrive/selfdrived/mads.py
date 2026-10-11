from opendbc.car.structs import car
from opendbc.safety import ALTERNATIVE_EXPERIENCE
from openpilot.cereal import log, custom
from openpilot.common.realtime import DT_CTRL
from openpilot.selfdrive.selfdrived.events import ET
from openpilot.selfdrive.selfdrived.state import SOFT_DISABLE_TIME

# MADS: lateral-only engagement, ported from sunnypilot and hard-coded for one configuration:
# ACC main engages lateral, engaging cruise also engages lateral, lateral stays active while braking
# and gas never disengages. Toyota only; the panda safety enforces the same rules.
# Mads.update() must run after the stock StateMachine.update(): MADS events such as lkasEnable carry
# ET.ENABLE and would engage openpilot itself if the stock state machine saw them.

State = custom.SelfdriveStateSP.Mads.State
EventName = log.OnroadEvent.EventName
ButtonType = car.CarState.ButtonEvent.Type
GearShifter = car.CarState.GearShifter
SafetyModel = car.CarParams.SafetyModel

ACTIVE_STATES = (State.enabled, State.softDisabling, State.overriding)
ENABLED_STATES = (State.paused, *ACTIVE_STATES)

PAUSE_EVENTS = (EventName.wrongGear, EventName.reverseGear, EventName.brakeHold,
                EventName.doorOpen, EventName.seatbeltNotLatched, EventName.parkBrake)
SILENT_PAUSE_EVENTS = (EventName.silentWrongGear, EventName.silentReverseGear, EventName.silentBrakeHold,
                       EventName.silentDoorOpen, EventName.silentSeatbeltNotLatched, EventName.silentParkBrake)
SET_SPEED_BUTTONS = (ButtonType.accelCruise, ButtonType.resumeCruise, ButtonType.decelCruise, ButtonType.setCruise)
LATERAL_ONLY_IGNORED_EVENTS = (EventName.preEnableStandstill, EventName.belowEngageSpeed, EventName.speedTooLow,
                               EventName.cruiseDisabled, EventName.manualRestart, EventName.espActive)
LONGITUDINAL_DISENGAGE_EVENTS = (EventName.pcmDisable, EventName.buttonCancel, EventName.pedalPressed,
                                 EventName.wrongCruiseMode)
IGNORED_SAFETY_MODES = (SafetyModel.silent, SafetyModel.noOutput)
LATERAL_MISMATCH_FRAMES = int(2. / DT_CTRL)


def set_mads_alternative_experience(CP) -> None:
  if CP.brand == "toyota":
    CP.alternativeExperience |= ALTERNATIVE_EXPERIENCE.ENABLE_MADS


def mads_available(CP) -> bool:
  return (CP.alternativeExperience & ALTERNATIVE_EXPERIENCE.ENABLE_MADS) != 0


def mads_lat_active(selfdrive_state, selfdrive_state_sp) -> bool:
  mads = selfdrive_state_sp.mads
  return mads.active if mads.available else selfdrive_state.active


class MadsStateMachine:
  def __init__(self):
    self.state = State.disabled

  def update(self, events, ss_sm, selfdrive_enabled: bool) -> tuple[bool, bool]:
    # soft disable timer and current alert types belong to the stock state machine (ss_sm)
    def add_alert_type(alert_type):
      if not selfdrive_enabled:
        ss_sm.current_alert_types.append(alert_type)

    if self.state != State.disabled:
      # immediate and user disable always have priority in a non-disabled state; immediate wins over a pause
      if events.contains(ET.IMMEDIATE_DISABLE):
        self.state = State.disabled
        add_alert_type(ET.IMMEDIATE_DISABLE)
      elif events.contains(ET.USER_DISABLE):
        self.state = State.paused if events.has(EventName.silentLkasDisable) else State.disabled
        ss_sm.current_alert_types.append(ET.USER_DISABLE)
      else:
        if self.state == State.enabled:
          if events.contains(ET.SOFT_DISABLE):
            self.state = State.softDisabling
            if not selfdrive_enabled:
              ss_sm.soft_disable_timer = int(SOFT_DISABLE_TIME / DT_CTRL)
              ss_sm.current_alert_types.append(ET.SOFT_DISABLE)
          elif events.contains(ET.OVERRIDE_LATERAL):
            self.state = State.overriding
            add_alert_type(ET.OVERRIDE_LATERAL)

        elif self.state == State.softDisabling:
          if not events.contains(ET.SOFT_DISABLE):
            self.state = State.enabled
          elif ss_sm.soft_disable_timer > 0:
            add_alert_type(ET.SOFT_DISABLE)
          else:
            self.state = State.disabled

        elif self.state == State.paused:
          if events.contains(ET.ENABLE):
            if events.contains(ET.NO_ENTRY):
              add_alert_type(ET.NO_ENTRY)
            else:
              self.state = State.overriding if events.contains(ET.OVERRIDE_LATERAL) else State.enabled
              add_alert_type(ET.ENABLE)

        elif self.state == State.overriding:
          if events.contains(ET.SOFT_DISABLE):
            self.state = State.softDisabling
            if not selfdrive_enabled:
              ss_sm.soft_disable_timer = int(SOFT_DISABLE_TIME / DT_CTRL)
              ss_sm.current_alert_types.append(ET.SOFT_DISABLE)
          elif not events.contains(ET.OVERRIDE_LATERAL):
            self.state = State.enabled
          else:
            ss_sm.current_alert_types.append(ET.OVERRIDE_LATERAL)

    elif events.contains(ET.ENABLE):
      if events.contains(ET.NO_ENTRY):
        if events.contains_in_list(PAUSE_EVENTS) or events.contains_in_list(SILENT_PAUSE_EVENTS):
          self.state = State.paused
        add_alert_type(ET.NO_ENTRY)
      else:
        self.state = State.overriding if events.contains(ET.OVERRIDE_LATERAL) else State.enabled
        add_alert_type(ET.ENABLE)

    enabled = self.state in ENABLED_STATES
    active = self.state in ACTIVE_STATES
    if active:
      add_alert_type(ET.WARNING)
    return enabled, active


class Mads:
  def __init__(self, CP):
    self.available = mads_available(CP)
    self.state_machine = MadsStateMachine()
    self.enabled = False
    self.active = False
    self.lateral_mismatch_counter = 0
    self.selfdrive_enabled_prev = False

  def update(self, CS, CS_prev, events, ss_sm, selfdrive_enabled: bool, panda_states, initialized: bool, passive: bool) -> None:
    if not self.available:
      return

    self._sample_lateral_mismatch(panda_states, selfdrive_enabled)
    self._update_events(CS, CS_prev, events, selfdrive_enabled)
    if not passive and initialized:
      self.enabled, self.active = self.state_machine.update(events, ss_sm, selfdrive_enabled)
    self.selfdrive_enabled_prev = selfdrive_enabled

  def _sample_lateral_mismatch(self, panda_states, selfdrive_enabled: bool) -> None:
    # The panda reports controls_allowed_lateral on another socket than CAN, so allow a short mismatch
    if not self.active or selfdrive_enabled:
      self.lateral_mismatch_counter = 0
    elif any(not ps.controlsAllowedLateral for ps in panda_states if ps.safetyModel not in IGNORED_SAFETY_MODES):
      self.lateral_mismatch_counter += 1

  def _pause_on(self, events, event, silent_event) -> None:
    if events.has(event):
      events.remove(event)
      events.add(silent_event)
      if self.state_machine.state != State.paused:
        events.add(EventName.silentLkasDisable)

  def _update_events(self, CS, CS_prev, events, selfdrive_enabled: bool) -> None:
    if not selfdrive_enabled and self.enabled:
      if CS.standstill:
        self._pause_on(events, EventName.doorOpen, EventName.silentDoorOpen)
        self._pause_on(events, EventName.seatbeltNotLatched, EventName.silentSeatbeltNotLatched)
      if CS.vEgo < 2.5 or CS.gearShifter == GearShifter.reverse:
        self._pause_on(events, EventName.wrongGear, EventName.silentWrongGear)
      self._pause_on(events, EventName.reverseGear, EventName.silentReverseGear)
      self._pause_on(events, EventName.brakeHold, EventName.silentBrakeHold)
      self._pause_on(events, EventName.parkBrake, EventName.silentParkBrake)
      for e in LATERAL_ONLY_IGNORED_EVENTS:
        events.remove(e)

    selfdrive_enable_events = events.has(EventName.pcmEnable) or events.has(EventName.buttonEnable)
    set_speed_btns_enable = any(be.type in SET_SPEED_BUTTONS for be in CS.buttonEvents)

    # wrongCarMode only alerts when the driver tries to engage cruise; it must not block lateral
    if events.has(EventName.wrongCarMode):
      events.remove(EventName.wrongCarMode)
      if selfdrive_enable_events or set_speed_btns_enable:
        events.add(EventName.wrongCarModeAlertOnly)

    if selfdrive_enable_events:
      if events.has(EventName.pedalPressed):
        events.add(EventName.pedalPressedAlertOnly)
      # unified engagement: engaging cruise also engages MADS, unless MADS or openpilot already was
      if self.enabled or (selfdrive_enabled and self.selfdrive_enabled_prev):
        events.remove(EventName.pcmEnable)
        events.remove(EventName.buttonEnable)
    elif CS.cruiseState.available and not CS_prev.cruiseState.available:
      events.add(EventName.lkasEnable)

    if not selfdrive_enabled and self.selfdrive_enabled_prev and any(be.type == ButtonType.cancel for be in CS.buttonEvents):
      events.add(EventName.manualLongitudinalRequired)

    if not CS.cruiseState.available:
      events.remove(EventName.buttonEnable)
      if CS_prev.cruiseState.available:
        events.add(EventName.lkasDisable)

    if self.state_machine.state == State.paused and not events.contains_in_list(SILENT_PAUSE_EVENTS):
      events.add(EventName.silentLkasEnable)

    if self.lateral_mismatch_counter >= LATERAL_MISMATCH_FRAMES:
      events.add(EventName.controlsMismatchLateral)

    # longitudinal disengagement never touches lateral (REMAIN_ACTIVE); the stock state machine already consumed these
    for e in LONGITUDINAL_DISENGAGE_EVENTS:
      events.remove(e)
