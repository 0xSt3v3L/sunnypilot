from openpilot.common.test import OpenpilotTestCase
from openpilot.cereal import log, custom
from openpilot.cereal.services import SERVICE_LIST
from openpilot.selfdrive.selfdrived.events import Events, EVENTS, ET

EventName = log.OnroadEvent.EventName

MADS_EVENTS = ("lkasEnable", "lkasDisable", "manualLongitudinalRequired", "silentLkasEnable", "silentLkasDisable",
               "silentBrakeHold", "silentWrongGear", "silentReverseGear", "silentDoorOpen", "silentSeatbeltNotLatched",
               "silentParkBrake", "controlsMismatchLateral", "wrongCarModeAlertOnly", "pedalPressedAlertOnly")


class TestMadsSchema(OpenpilotTestCase):
  def test_selfdrive_state_sp(self):
    states = list(custom.SelfdriveStateSP.Mads.State.schema.enumerants.keys())
    assert states == ["disabled", "paused", "enabled", "softDisabling", "overriding"]
    msg = log.Event.new_message(selfdriveStateSP={"mads": {"enabled": True, "active": True, "available": True}})
    assert msg.selfdriveStateSP.mads.enabled
    assert SERVICE_LIST["selfdriveStateSP"].frequency == 100.

  def test_panda_state_lateral(self):
    assert log.PandaState.new_message(controlsAllowedLateral=True).controlsAllowedLateral

  def test_mads_events_defined(self):
    enumerants = EventName.schema.enumerants
    for name in MADS_EVENTS:
      assert name in enumerants, name
      assert enumerants[name] in EVENTS, name
    assert ET.ENABLE in EVENTS[EventName.lkasEnable]
    assert ET.USER_DISABLE in EVENTS[EventName.lkasDisable]
    assert ET.IMMEDIATE_DISABLE in EVENTS[EventName.controlsMismatchLateral]
    assert ET.NO_ENTRY in EVENTS[EventName.controlsMismatchLateral]

  def test_events_helpers(self):
    events = Events()
    events.add(EventName.pcmEnable)
    events.add(EventName.pcmEnable)
    events.add(EventName.doorOpen)
    assert events.has(EventName.pcmEnable)
    assert events.contains_in_list([EventName.parkBrake, EventName.doorOpen])
    assert not events.contains_in_list([EventName.parkBrake])
    events.remove(EventName.pcmEnable)
    assert not events.has(EventName.pcmEnable)
    events.remove(EventName.parkBrake)  # removing an absent event is a no-op
    assert events.names == [EventName.doorOpen]
