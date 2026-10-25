from opendbc.car import gen_empty_fingerprint
from opendbc.car.car_helpers import interfaces
from opendbc.car.structs import car
from opendbc.car.toyota.eps_probe import with_eps_probe
from opendbc.car.toyota.values import CAR
from openpilot.common.test import OpenpilotTestCase
from openpilot.tools.op_mads.eps_probe import (DEFAULT_LEVELS, abort_reason, build_schedule, level_verdict, levels_from_args,
                                               probe_ready)


def corolla_ci():
  CarInterface = interfaces[CAR.TOYOTA_COROLLA_TSS2]
  return CarInterface(CarInterface.get_params(CAR.TOYOTA_COROLLA_TSS2, gen_empty_fingerprint(), [], False, False, docs=False))


def cs(**kw):
  return car.CarState.new_message(**kw)


class TestEpsProbeSchedule(OpenpilotTestCase):
  def test_default_schedule(self):
    steps = build_schedule(DEFAULT_LEVELS)
    assert len(steps) == 24
    applies = [s for s in steps if s.direction != "rest"]
    assert [s.target for s in applies] == [1500, -1500, 1800, -1800, 2100, -2100, 2400, -2400, 2700, -2700, 3000, -3000]
    assert [s.direction for s in applies[:2]] == ["left", "right"]
    assert all(s.target == 0 and s.duration == 2.5 for s in steps if s.direction == "rest")
    assert steps[0].duration == 1500 / 1500 + 0.5 + 1.5 and steps[0].hold_s == 1.5
    assert abs(sum(s.duration for s in steps) - 72.) < 0.01

  def test_max_cuts_levels(self):
    assert levels_from_args(None, 2100) == [1500, 1800, 2100]
    assert levels_from_args([1500, 2000], None) == [1500, 2000]
    assert levels_from_args(None, None) == DEFAULT_LEVELS


class TestEpsProbeAbort(OpenpilotTestCase):
  def test_nominal(self):
    assert abort_reason(cs(), True, 1, 0.) is None
    assert abort_reason(cs(vEgo=2 / 3.6), True, 2, 0.) is None

  def test_abort_reasons(self):
    cases = [
      (cs(steerFaultTemporary=True), 1, 0.),
      (cs(steerFaultPermanent=True), 1, 0.),
      (cs(steeringPressed=True), 1, 0.),
      (cs(steeringRateDeg=101.), 1, 0.),
      (cs(steeringAngleDeg=-451.), 1, 0.),
      (cs(vEgo=0.31), 1, 0.),
      (cs(vEgo=5.1 / 3.6), 2, 0.),
      (cs(), 1, 0.51),
    ]
    for state, stage, lat_lost_s in cases:
      reason = abort_reason(state, True, stage, lat_lost_s)
      assert reason, (state, stage, lat_lost_s)


class TestEpsProbeVerdict(OpenpilotTestCase):
  def test_verdicts(self):
    assert level_verdict(2100, [2100.] * 10, [3045.] * 10, False) == "ACCEPT"
    assert level_verdict(-2100, [-2100.] * 10, [-3045.] * 10, False) == "ACCEPT"
    assert level_verdict(3000, [2550.] * 10, [2200.] * 10, False) == "CLIP"
    assert level_verdict(3000, [3000.] * 10, [2200.] * 10, False) == "CLIP"
    assert level_verdict(1500, [1500.] * 10, [20.] * 10, False) == "NO-RESPONSE"
    assert level_verdict(2100, [2100.] * 10, [3045.] * 10, True) == "FAULT"
    assert level_verdict(2100, [], [], False) == "NO-RESPONSE"

  def test_refuses_without_probe(self):
    CI = corolla_ci()
    assert not probe_ready(CI.CP)
    assert probe_ready(with_eps_probe(CI).CP)
