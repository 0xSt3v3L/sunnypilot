from opendbc.car.structs import car
from opendbc.safety import ALTERNATIVE_EXPERIENCE
from openpilot.cereal import log
from openpilot.common.realtime import DT_CTRL
from openpilot.common.test import OpenpilotTestCase
from openpilot.selfdrive.selfdrived.events import Events, ET, EVENTS, NormalPermanentAlert
from openpilot.selfdrive.selfdrived.state import StateMachine, SOFT_DISABLE_TIME
from openpilot.selfdrive.selfdrived.mads import (Mads, MadsStateMachine, State, LATERAL_MISMATCH_FRAMES,
                                                 set_mads_alternative_experience, mads_available)

EventName = log.OnroadEvent.EventName
OpenpilotState = log.SelfdriveState.OpenpilotState

MAINTAIN_STATES = {State.enabled: (None,), State.disabled: (None,), State.softDisabling: (ET.SOFT_DISABLE,),
                   State.paused: (None,), State.overriding: (ET.OVERRIDE_LATERAL,)}
ALL_STATES = tuple(State.schema.enumerants.values())
ENABLE_EVENT_TYPES = (ET.ENABLE, ET.OVERRIDE_LATERAL)


def make_event(event_types):
  EVENTS[0] = {}
  for ev in event_types:
    if ev is not None:
      EVENTS[0][ev] = NormalPermanentAlert("alert")
  return 0


def mads_cp(brand="toyota"):
  CP = car.CarParams.new_message(brand=brand)
  set_mads_alternative_experience(CP)
  return CP


def car_state(main=True, **kwargs):
  return car.CarState.new_message(cruiseState={"available": main}, **kwargs)


def panda_state(lateral):
  return log.PandaState.new_message(controlsAllowedLateral=lateral, safetyModel=car.CarParams.SafetyModel.toyota)


class TestMadsStateMachine(OpenpilotTestCase):
  def setup_method(self):
    self.events = Events()
    self.ss_sm = StateMachine()
    self.ss_sm.soft_disable_timer = int(SOFT_DISABLE_TIME / DT_CTRL)
    self.sm = MadsStateMachine()

  def update(self, selfdrive_enabled=False):
    return self.sm.update(self.events, self.ss_sm, selfdrive_enabled)

  def test_immediate_disable(self):
    for state in ALL_STATES:
      for et in MAINTAIN_STATES[state]:
        self.events.add(make_event([et, ET.IMMEDIATE_DISABLE]))
        self.sm.state = state
        self.update()
        assert self.sm.state == State.disabled
        self.events.clear()

  def test_user_disable(self):
    for state in ALL_STATES:
      for et in MAINTAIN_STATES[state]:
        self.events.add(make_event([et, ET.USER_DISABLE]))
        self.sm.state = state
        self.update()
        assert self.sm.state == State.disabled
        self.events.clear()

  def test_user_disable_to_paused(self):
    for state in ALL_STATES:
      for et in MAINTAIN_STATES[state]:
        self.events.add(make_event([et, ET.USER_DISABLE]))
        self.events.add(EventName.silentLkasDisable)
        self.sm.state = state
        self.update()
        assert self.sm.state == (State.disabled if state == State.disabled else State.paused)
        self.events.clear()

  def test_soft_disable_timer(self):
    self.sm.state = State.enabled
    self.events.add(make_event([ET.SOFT_DISABLE]))
    self.update()
    for _ in range(int(SOFT_DISABLE_TIME / DT_CTRL)):
      assert self.sm.state == State.softDisabling
      self.ss_sm.soft_disable_timer -= 1
      self.update()
    assert self.sm.state == State.disabled

  def test_no_entry(self):
    for et in ENABLE_EVENT_TYPES:
      self.events.add(make_event([ET.NO_ENTRY, et]))
      self.update()
      assert self.sm.state == State.disabled
      self.events.clear()

  def test_no_entry_with_pause_event_pauses(self):
    self.events.add(make_event([ET.ENABLE]))
    self.events.add(EventName.doorOpen)
    self.update()
    assert self.sm.state == State.paused

  def test_override_lateral(self):
    self.sm.state = State.enabled
    self.events.add(make_event([ET.OVERRIDE_LATERAL]))
    self.update()
    assert self.sm.state == State.overriding

  def test_paused_to_enabled(self):
    self.sm.state = State.paused
    self.events.add(make_event([ET.ENABLE]))
    self.update()
    assert self.sm.state == State.enabled

  def test_maintain_states(self):
    for state in ALL_STATES:
      for et in MAINTAIN_STATES[state]:
        self.sm.state = state
        self.events.add(make_event([et]))
        self.update()
        assert self.sm.state == state
        self.events.clear()

  def test_active_lateral_only_adds_warning_alerts(self):
    self.sm.state = State.enabled
    self.ss_sm.current_alert_types = [ET.PERMANENT]
    enabled, active = self.update(selfdrive_enabled=False)
    assert enabled and active
    assert ET.WARNING in self.ss_sm.current_alert_types


class TestMads(OpenpilotTestCase):
  def setup_method(self):
    self.events = Events()
    self.ss_sm = StateMachine()
    self.mads = Mads(mads_cp())
    self.CS_prev = car_state(main=False)
    self.selfdrive_enabled = False

  def step(self, CS, events=(), lateral=True, initialized=True):
    """One selfdrived step in the real order: stock events, stock state machine, then MADS."""
    self.events.clear()
    for e in events:
      self.events.add(e)
    if initialized:
      self.selfdrive_enabled, _ = self.ss_sm.update(self.events)
    self.mads.update(CS, self.CS_prev, self.events, self.ss_sm, self.selfdrive_enabled, [panda_state(lateral)],
                     initialized, False)
    self.CS_prev = CS

  def test_alternative_experience_toyota_only(self):
    assert mads_available(mads_cp("toyota"))
    assert mads_cp("toyota").alternativeExperience & ALTERNATIVE_EXPERIENCE.ENABLE_MADS
    assert not mads_available(mads_cp("honda"))

  def test_main_on_enables_lateral_only(self):
    self.step(car_state(main=True))
    assert self.events.has(EventName.lkasEnable)
    assert self.mads.enabled and self.mads.active
    assert not self.selfdrive_enabled

  def test_lkas_enable_does_not_engage_openpilot(self):
    self.step(car_state(main=True))
    for _ in range(5):
      self.step(car_state(main=True))
    assert self.mads.active
    assert not self.selfdrive_enabled
    assert self.ss_sm.state == OpenpilotState.disabled

  def test_main_off_disables(self):
    self.step(car_state(main=True))
    self.step(car_state(main=False))
    assert self.events.has(EventName.lkasDisable)
    assert not self.mads.enabled

  def test_cruise_engage_enables_mads(self):
    self.CS_prev = car_state(main=True)
    self.step(car_state(main=True), events=[EventName.pcmEnable])
    assert self.selfdrive_enabled
    assert self.mads.enabled and self.mads.active

  def test_brake_keeps_mads_and_hides_disengage(self):
    self.CS_prev = car_state(main=True)
    self.step(car_state(main=True), events=[EventName.pcmEnable])
    self.step(car_state(main=True, brakePressed=True), events=[EventName.pedalPressed, EventName.pcmDisable])
    assert not self.selfdrive_enabled
    assert self.mads.active
    assert not self.events.has(EventName.pedalPressed)
    assert not self.events.has(EventName.pcmDisable)

  def test_cancel_button_warns_manual_longitudinal(self):
    self.CS_prev = car_state(main=True)
    self.step(car_state(main=True), events=[EventName.pcmEnable])
    cancel = car_state(main=True, buttonEvents=[{"type": car.CarState.ButtonEvent.Type.cancel, "pressed": True}])
    self.step(cancel, events=[EventName.buttonCancel])
    assert self.events.has(EventName.manualLongitudinalRequired)
    assert self.mads.active

  def test_door_open_at_standstill_pauses_and_resumes(self):
    self.step(car_state(main=True))
    self.step(car_state(main=True, standstill=True), events=[EventName.doorOpen])
    assert self.events.has(EventName.silentDoorOpen)
    assert not self.events.has(EventName.doorOpen)
    assert self.mads.state_machine.state == State.paused
    assert self.mads.enabled and not self.mads.active
    self.step(car_state(main=True))
    assert self.events.has(EventName.silentLkasEnable)
    assert self.mads.active

  def test_lateral_mismatch_disables_after_two_seconds(self):
    self.step(car_state(main=True))
    for _ in range(LATERAL_MISMATCH_FRAMES - 1):
      self.step(car_state(main=True), lateral=False)
      assert self.mads.active
    self.step(car_state(main=True), lateral=False)
    assert self.events.has(EventName.controlsMismatchLateral)
    assert not self.mads.enabled

  def trigger_lateral_mismatch(self):
    self.step(car_state(main=True))
    for _ in range(LATERAL_MISMATCH_FRAMES):
      self.step(car_state(main=True), lateral=False)
    assert self.events.has(EventName.controlsMismatchLateral)
    assert not self.mads.enabled

  def test_lateral_mismatch_forbids_reenable_on_main_toggle(self):
    self.trigger_lateral_mismatch()
    self.step(car_state(main=True), lateral=True)
    self.step(car_state(main=False), lateral=True)
    self.step(car_state(main=True), lateral=True)
    assert not self.mads.enabled
    assert not self.events.has(EventName.lkasEnable)
    assert not self.events.has(EventName.controlsMismatchLateral)

  def test_lateral_mismatch_keeps_stock_cruise_engagement(self):
    self.trigger_lateral_mismatch()
    self.step(car_state(main=True), lateral=True)
    self.step(car_state(main=True), events=[EventName.pcmEnable], lateral=True)
    assert self.selfdrive_enabled
    assert not self.mads.enabled and not self.mads.active
    self.step(car_state(main=True), lateral=True)
    assert self.selfdrive_enabled
    assert not self.mads.enabled

  def test_fresh_mads_is_not_latched(self):
    self.trigger_lateral_mismatch()
    self.mads = Mads(mads_cp())
    self.CS_prev = car_state(main=False)
    self.step(car_state(main=True))
    assert self.mads.enabled and self.mads.active

  def test_main_on_before_initialized_does_not_enable(self):
    self.step(car_state(main=True), initialized=False)
    self.step(car_state(main=True), initialized=True)
    assert not self.mads.enabled
    assert not self.events.has(EventName.controlsMismatchLateral)

  def test_unavailable_is_inert(self):
    self.mads = Mads(mads_cp("honda"))
    self.step(car_state(main=True))
    assert not self.mads.enabled
    assert not self.events.has(EventName.lkasEnable)


class TestMadsIntegration(OpenpilotTestCase):
  def test_selfdrived_wires_mads(self):
    import inspect
    from openpilot.selfdrive.selfdrived import selfdrived
    src = inspect.getsource(selfdrived.SelfdriveD)
    assert "self.mads = Mads(self.CP)" in src
    step = inspect.getsource(selfdrived.SelfdriveD.step)
    # MADS must run after the stock state machine and before alerts are created
    assert step.index("self.state_machine.update(") < step.index("self.mads.update(") < step.index("self.update_alerts(")
    assert "'selfdriveStateSP'" in src

  def test_selfdrived_publishes_selfdriveStateSP_first(self):
    import inspect
    from openpilot.selfdrive.selfdrived import selfdrived
    src = inspect.getsource(selfdrived.SelfdriveD.publish_selfdriveState)
    # controlsd polls on selfdriveState, so selfdriveStateSP must already be out when it wakes
    assert src.index("self.pm.send('selfdriveStateSP'") < src.index("self.pm.send('selfdriveState'")

  def test_big_model_failure_treats_mads_like_enabled(self):
    # A behavioral test would need a full SelfdriveD (SubMaster, Params, CarParams); assert the condition text instead.
    import inspect
    from openpilot.selfdrive.selfdrived import selfdrived
    src = inspect.getsource(selfdrived.SelfdriveD.update_events)
    assert "if not (self.enabled or self.mads.enabled) and not model_unavailable:" in src

  def test_card_sets_mads_alternative_experience(self):
    import inspect
    from openpilot.selfdrive.car import card
    src = inspect.getsource(card)
    assert src.index("self.CP.alternativeExperience = 0") < src.index("set_mads_alternative_experience(self.CP)")


class TestMadsLatActive(OpenpilotTestCase):
  def test_lat_active_follows_mads(self):
    from openpilot.cereal import custom
    from openpilot.selfdrive.selfdrived.mads import mads_lat_active
    ss = log.SelfdriveState.new_message(active=False)
    ss_sp = custom.SelfdriveStateSP.new_message(mads={"available": True, "active": True})
    assert mads_lat_active(ss, ss_sp)
    ss_sp.mads.active = False
    ss.active = True
    assert not mads_lat_active(ss, ss_sp)

  def test_lat_active_falls_back_without_mads(self):
    from openpilot.cereal import custom
    from openpilot.selfdrive.selfdrived.mads import mads_lat_active
    ss = log.SelfdriveState.new_message(active=True)
    ss_sp = custom.SelfdriveStateSP.new_message()
    assert mads_lat_active(ss, ss_sp)

  def test_controlsd_uses_mads_lat_active(self):
    import inspect
    from openpilot.selfdrive.controls import controlsd
    src = inspect.getsource(controlsd)
    assert "mads_lat_active(self.sm['selfdriveState'], self.sm['selfdriveStateSP'])" in src
    assert "'selfdriveStateSP'" in src
    assert "if self.sm['selfdriveState'].active:" not in src


class TestPandadMads(OpenpilotTestCase):
  def test_pandad_sends_mads_heartbeat_and_reports_lateral(self):
    from openpilot.common.basedir import BASEDIR
    import os
    pandad = open(os.path.join(BASEDIR, "openpilot/selfdrive/pandad/pandad.cc")).read()
    panda_cc = open(os.path.join(BASEDIR, "openpilot/selfdrive/pandad/panda.cc")).read()
    assert 'SubMaster sm({"selfdriveState", "selfdriveStateSP", "deviceState"});' in pandad
    assert 'sm.allAliveAndValid({"selfdriveStateSP"}) && sm["selfdriveStateSP"].getSelfdriveStateSP().getMads().getEnabled()' in pandad
    assert "panda->send_heartbeat(engaged, engaged_mads);" in pandad
    assert "ps.setControlsAllowedLateral((health.flags_pkt & HEALTH_FLAG_CONTROLS_ALLOWED_LATERAL) != 0U);" in pandad
    assert "handle->control_write(0xf3, engaged, engaged_mads);" in panda_cc
