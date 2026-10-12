import os
import unittest
from opendbc.car.structs import car
from opendbc.safety import ALTERNATIVE_EXPERIENCE
from openpilot.cereal import log, custom, messaging
from openpilot.common.params import Params
from openpilot.common.test import OpenpilotTestCase
from openpilot.selfdrive.test.process_replay.process_replay import get_process_config, get_car_params_callback
from openpilot.selfdrive.test.process_replay.mads_replay import (transitions, compare_transitions, init_end_time, mads_state, enabled,
                                                                 mismatch_events, standstill_alert_violations, run_route)

MadsState = custom.SelfdriveStateSP.Mads.State
EventName = log.OnroadEvent.EventName
ROUTE = os.environ.get("MADS_REPLAY_ROUTE")


def at(t, **kwargs):
  msg = log.Event.new_message(**kwargs)
  msg.logMonoTime = int(t * 1e9)
  return msg.as_reader()


def sp(t, state):
  return at(t, selfdriveStateSP={"mads": {"state": state, "available": True}})


def ss(t, is_enabled=False):
  return at(t, selfdriveState={"enabled": is_enabled})


def cs(t, standstill=True):
  return at(t, carState={"standstill": standstill, "vEgo": 0. if standstill else 5.})


def events(t, *names):
  return at(t, onroadEvents=[{"name": n} for n in names])


class TestTransitions(OpenpilotTestCase):
  def test_transitions_keep_changes_only(self):
    msgs = [sp(0., MadsState.disabled), sp(1., MadsState.disabled), sp(2., MadsState.enabled), ss(2.5), sp(3., MadsState.paused)]
    assert transitions(msgs, "selfdriveStateSP", mads_state) == [(0., "disabled"), (2., "enabled"), (3., "paused")]

  def test_enabled_transitions(self):
    assert transitions([ss(0.), ss(1., True)], "selfdriveState", enabled) == [(0., False), (1., True)]

  def test_compare_identical(self):
    a = [(0., "disabled"), (2., "enabled")]
    assert compare_transitions(a, a, start=0.) == []

  def test_compare_within_tolerance(self):
    assert compare_transitions([(2., "enabled")], [(2.03, "enabled")], start=0.) == []

  def test_compare_late_transition(self):
    assert len(compare_transitions([(2., "enabled")], [(2.5, "enabled")], start=0.)) == 1

  def test_compare_different_sequence(self):
    assert len(compare_transitions([(2., "enabled"), (3., "paused")], [(2., "enabled")], start=0.)) == 1

  def test_compare_ignores_before_start(self):
    assert compare_transitions([(0., "disabled"), (5., "enabled")], [(0.4, "enabled"), (5., "enabled")], start=1.) == []

  def test_init_end_time(self):
    msgs = [events(0.5, EventName.selfdriveInitializing), events(3., EventName.selfdriveInitializing), events(4., EventName.startup)]
    assert init_end_time(msgs) == 3.
    assert init_end_time([events(1., EventName.startup)]) == 0.


class TestEventChecks(OpenpilotTestCase):
  def test_mismatch_events(self):
    msgs = [events(1., EventName.steerOverride), events(2., EventName.controlsMismatchLateral)]
    assert mismatch_events(msgs) == [2.]

  def test_standstill_alert_ok(self):
    assert standstill_alert_violations([cs(0.), ss(0.), events(1., EventName.greenLight)]) == []

  def test_standstill_alert_while_moving(self):
    assert len(standstill_alert_violations([cs(0., standstill=False), ss(0.), events(1., EventName.leadDeparted)])) == 1

  def test_standstill_alert_while_engaged(self):
    assert len(standstill_alert_violations([cs(0.), ss(0., True), events(1., EventName.greenLight)])) == 1


class TestReplayConfig(OpenpilotTestCase):
  def test_selfdrived_publishes_mads_state(self):
    assert "selfdriveStateSP" in get_process_config("selfdrived").subs

  def test_car_params_enable_mads(self):
    get_car_params_callback(None, None, [], "TOYOTA_COROLLA_TSS2")
    CP = messaging.log_from_bytes(Params().get("CarParams"), car.CarParams)
    assert CP.alternativeExperience & ALTERNATIVE_EXPERIENCE.ENABLE_MADS


@unittest.skipUnless(ROUTE, "set MADS_REPLAY_ROUTE to a local route directory with <segment>/rlog.zst")
class TestMadsRouteReplay(OpenpilotTestCase):
  def test_route_matches_logged_run(self):
    report = run_route(ROUTE)
    assert report["problems"] == [], "\n".join(report["problems"])
