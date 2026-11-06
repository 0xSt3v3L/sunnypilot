from openpilot.cereal import log
from openpilot.common.test import OpenpilotTestCase
from openpilot.tools.op_mads.legacy_events import event_name, is_legacy_log

EventName = log.OnroadEvent.EventName


def event(raw):
  return log.OnroadEvent.new_message(name=raw).as_reader()


def init_data(os_version):
  return log.Event.new_message(initData={"osVersion": os_version}).as_reader()


class TestLegacyEvents(OpenpilotTestCase):
  def test_legacy_log_by_agnos(self):
    assert is_legacy_log([init_data("19.9\n")])
    assert not is_legacy_log([init_data("20.3\n")])
    assert not is_legacy_log([init_data("")])
    assert not is_legacy_log([])

  def test_legacy_ordinals(self):
    # builds before the 89e7e0a4c4 merge logged the fork events at @105-@124
    assert event_name(event(105), legacy=True) == "lkasEnable"
    assert event_name(event(116), legacy=True) == "controlsMismatchLateral"
    assert event_name(event(119), legacy=True) == "greenLight"
    assert event_name(event(124), legacy=True) == "laneChangeRoadEdge"
    assert event_name(event(EventName.steerSaturated), legacy=True) == "steerSaturated"

  def test_current_ordinals(self):
    assert event_name(event(105), legacy=False) == "startupCruiseCanceled"
    assert event_name(event(EventName.lkasEnable), legacy=False) == "lkasEnable"
    assert event_name(event(EventName.laneChangeRoadEdge), legacy=False) == "laneChangeRoadEdge"
