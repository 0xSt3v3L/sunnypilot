from opendbc.car import gen_empty_fingerprint
from opendbc.car.car_helpers import interfaces
from opendbc.car.structs import car
from opendbc.car.toyota.eps_probe import with_eps_probe
from opendbc.car.toyota.values import CAR
from openpilot.common.test import OpenpilotTestCase
import os
import signal
import tempfile
from pathlib import Path

from openpilot.tools.op_mads.eps_probe import (DEFAULT_LEVELS, DEFAULT_RATES, NO_LATERAL, abort_reason, acquire_lock,
                                               build_schedule, build_sweep, install_signal_handlers, level_verdict, levels_from_args,
                                               probe_ready, rates_from_args, stage2_decision, sweep_summary,
                                               transition_rate, tx_blocked_reason)


def corolla_ci():
  CarInterface = interfaces[CAR.TOYOTA_COROLLA_TSS2]
  return CarInterface(CarInterface.get_params(CAR.TOYOTA_COROLLA_TSS2, gen_empty_fingerprint(), [], False, False, docs=False))


def cs(**kw):
  return car.CarState.new_message(**kw)


class TestEpsProbeSchedule(OpenpilotTestCase):
  def test_default_schedule(self):
    # the EPS faults above 1500 (route 000000d1): level mode only steps to 1500 now
    assert DEFAULT_LEVELS == [1500]
    steps = build_schedule(DEFAULT_LEVELS)
    assert [s.target for s in steps] == [1500, 0, -1500, 0]
    assert [s.direction for s in steps] == ["left", "rest", "right", "rest"]
    assert all(s.rate == 0 for s in steps)
    assert steps[0].duration == 1500 / 1500 + 0.5 + 1.5 and steps[0].hold_s == 1.5

  def test_max_cuts_levels(self):
    assert levels_from_args(None, 1200) == []
    assert levels_from_args([900, 1500], None) == [900, 1500]
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


class TestEpsProbeReviewFixes(OpenpilotTestCase):
  def test_panda_tx_blocked_is_not_an_eps_result(self):
    assert tx_blocked_reason(3, 3) is None
    reason = tx_blocked_reason(3, 4)
    assert reason is not None and "panda" in reason and "not an EPS result" in reason

  def test_sighup_and_sigterm_raise_keyboard_interrupt(self):
    old = {s: signal.getsignal(s) for s in (signal.SIGHUP, signal.SIGTERM)}
    try:
      install_signal_handlers()
      for sig in (signal.SIGHUP, signal.SIGTERM):
        try:
          os.kill(os.getpid(), sig)
          raise AssertionError("no KeyboardInterrupt")
        except KeyboardInterrupt:
          pass
    finally:
      for sig, handler in old.items():
        signal.signal(sig, handler)

  def test_second_instance_is_refused(self):
    path = Path(tempfile.mkdtemp()) / "lock"
    first = acquire_lock(path)
    assert first is not None
    assert acquire_lock(path) is None
    first.close()
    assert acquire_lock(path) is not None

  def test_stage2_decision(self):
    accept = [(lv, "ACCEPT") for lv in (1500, 1800, 2100)]
    assert stage2_decision(1, accept, None).startswith("stage 2: GO")
    assert stage2_decision(1, [], NO_LATERAL).startswith("stage 2: GO")
    assert stage2_decision(1, [(1500, "NO-RESPONSE"), (1500, "NO-RESPONSE")], None).startswith("stage 2: GO")
    fault_1500 = stage2_decision(1, [(1500, "FAULT")], "steer fault")
    assert fault_1500.startswith("stage 2: GO") and "JoystickDebugMode" in fault_1500
    assert stage2_decision(1, [(1500, "ACCEPT"), (1500, "ACCEPT"), (1800, "FAULT")], "steer fault").startswith("stage 2: NO")
    assert stage2_decision(1, [(1500, "ACCEPT"), (1500, "ACCEPT"), (1800, "CLIP"), (1800, "CLIP")], None).startswith("stage 2: NO")
    assert stage2_decision(1, [(1500, "CLIP"), (1500, "CLIP")], None).startswith("stage 2: UNDECIDED")
    assert stage2_decision(1, [(1500, "ACCEPT"), (1500, "NO-RESPONSE")], None).startswith("stage 2: UNDECIDED")
    assert stage2_decision(1, [(1500, "ACCEPT")], "driver steering").startswith("stage 2: UNDECIDED")
    assert stage2_decision(1, [(1500, "ACCEPT")], tx_blocked_reason(0, 1)).startswith("stage 2: UNDECIDED")
    assert stage2_decision(2, accept, None) == ""



class TestEpsProbeEntryPoint(OpenpilotTestCase):
  def test_repo_root_on_path(self):
    # on the device an SSH shell has no PYTHONPATH (manager gets /data/openpilot from launch_chffrplus.sh)
    from openpilot.tools.op_mads.eps_probe import REPO_ROOT, ensure_repo_on_path
    assert (REPO_ROOT / "openpilot" / "tools" / "op_mads" / "eps_probe.py").is_file()
    path: list[str] = ["/usr/lib/python3"]
    ensure_repo_on_path(path)
    ensure_repo_on_path(path)
    assert path == [str(REPO_ROOT), "/usr/lib/python3"]


class TestEpsProbeRateSweep(OpenpilotTestCase):
  def test_sweep_schedule(self):
    steps = build_sweep([30, 90])
    assert [s.direction for s in steps[:7]] == ["left", "rest", "right", "rest", "left", "reverse", "rest"]
    assert [s.target for s in steps[:7]] == [1500, 0, -1500, 0, 1500, -1500, 0]
    assert {s.rate for s in steps[:7]} == {30} and {s.rate for s in steps[7:]} == {90}
    left, rest, _, _, _, reverse, last = steps[:7]
    assert abs(left.duration - (0.5 + 0.3 + 1.0)) < 1e-9 and left.hold_s == 1.0
    assert abs(rest.duration - (0.5 + 1.5)) < 1e-9 and rest.hold_s == 0.
    assert abs(reverse.duration - (1.0 + 0.3 + 1.0)) < 1e-9 and reverse.hold_s == 1.0
    assert abs(last.duration - (0.5 + 2.0)) < 1e-9
    assert DEFAULT_RATES == [30, 45, 60, 75, 90]

  def test_joystick_axes(self):
    # the target jumps; the car controller ramps at the rate in axis 2 (card applies it in probe mode)
    from openpilot.tools.op_mads.eps_probe import LEVEL_RATE, joystick_axes
    assert joystick_axes(1500, 45) == [0., 1., 45.]
    assert joystick_axes(-750, 90) == [0., -0.5, 90.]
    assert joystick_axes(1500, 0) == [0., 1., float(LEVEL_RATE)] and LEVEL_RATE == 24

  def test_transition_rate(self):
    ramp = [float(min(45 * i, 1500)) for i in range(60)]
    assert abs(transition_rate(ramp, 0, 1500) - 45.) < 1.
    # message timing: some frames repeat, the next one catches up
    jitter = [0., 90., 90., 180., 225., 225., 315., 360.] + [float(min(360 + 45 * i, 1500)) for i in range(1, 40)]
    assert 38. < transition_rate(jitter, 0, 1500) < 48.
    down = [float(max(1500 - 60 * i, -1500)) for i in range(70)]
    assert abs(transition_rate(down, 1500, -1500) - 60.) < 1.
    assert transition_rate([0.] * 50, 0, 1500) == 0.

  def test_rates_from_args(self):
    assert rates_from_args(None) == DEFAULT_RATES
    assert rates_from_args([45, 60]) == [45, 60]
    for bad in ([0], [91], [-30]):
      try:
        rates_from_args(bad)
        raise AssertionError(bad)
      except ValueError:
        pass

  def test_sweep_summary(self):
    assert sweep_summary([(30, "ACCEPT"), (45, "ACCEPT"), (60, "FAULT")], None) == "highest rate without a fault: 45 (faulted at 60)"
    assert sweep_summary([(r, "ACCEPT") for r in (30, 45, 60, 75, 90)], None) == "highest rate without a fault: 90"
    assert sweep_summary([(30, "FAULT")], None) == "faulted at 30: no rate passed"
    assert sweep_summary([(30, "ACCEPT")], "driver steering").startswith("stopped early (driver steering)")

  def test_rate_max_matches_opendbc(self):
    from opendbc.car.toyota.eps_probe import EPS_PROBE_RATE_MAX
    from opendbc.car.toyota.eps_probe import EPS_PROBE_STEER_MAX
    from openpilot.tools.op_mads.eps_probe import RATE_MAX, SWEEP_LEVEL
    assert RATE_MAX == EPS_PROBE_RATE_MAX
    assert SWEEP_LEVEL == EPS_PROBE_STEER_MAX  # joystick axis 1 is a fraction of the probe-mode STEER_MAX

  def test_reach_time(self):
    from openpilot.tools.op_mads.eps_probe import reach_time
    assert reach_time([0., 900., 1800., 2100.], 2000.) == 0.03
    assert reach_time([0., -900., -2100.], -2000.) == 0.02
    assert reach_time([0., 900.], 2000.) is None

  def test_max_step(self):
    from openpilot.tools.op_mads.eps_probe import max_step
    assert max_step([0., 45., 45., 135., 180.]) == 90.
    assert max_step([1500., 1455., 1410.]) == 45.
    assert max_step([0.]) == 0.
