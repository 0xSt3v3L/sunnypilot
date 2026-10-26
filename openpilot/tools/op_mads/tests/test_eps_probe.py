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

from openpilot.tools.op_mads.eps_probe import (DEFAULT_LEVELS, NO_LATERAL, abort_reason, acquire_lock, build_schedule,
                                               install_signal_handlers, level_verdict, levels_from_args, probe_ready, stage2_decision,
                                               tx_blocked_reason)


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
