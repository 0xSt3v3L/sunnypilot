import unittest

from opendbc.car import gen_empty_fingerprint, structs
from opendbc.car.car_helpers import interfaces
from opendbc.car.hyundai.values import CAR as HYUNDAI
from opendbc.car.toyota.eps_probe import EPS_PROBE_STEER_MAX, eps_probe_active, with_eps_probe
from opendbc.car.toyota.values import CAR, CarControllerParams, ToyotaFlags, ToyotaSafetyFlags


def make_ci(candidate):
  CarInterface = interfaces[candidate]
  CP = CarInterface.get_params(candidate, gen_empty_fingerprint(), [], False, False, docs=False)
  return CarInterface(CP)


def final_torque(CI, frames=300):
  CI.CS.out.steeringTorqueEps = 5000.  # far above any command: the 350 command-vs-EPS limit never binds
  CC = structs.CarControl()
  CC.latActive = True
  CC.actuators.torque = 1.
  CC = CC.as_reader()
  out = None
  for i in range(frames):
    out, _ = CI.apply(CC, i * 10_000_000)
  return out.torqueOutputCan


class TestEpsProbe(unittest.TestCase):
  def test_default_limits(self):
    CI = make_ci(CAR.TOYOTA_COROLLA_TSS2)
    self.assertEqual(CarControllerParams(CI.CP).STEER_MAX, 1500)
    self.assertFalse(eps_probe_active(CI.CP))

  def test_with_eps_probe_sets_both_flags(self):
    CI = make_ci(CAR.TOYOTA_COROLLA_TSS2)
    probe = with_eps_probe(CI)
    self.assertIsNot(probe, CI)
    self.assertTrue(probe.CP.flags & ToyotaFlags.EPS_PROBE)
    self.assertTrue(probe.CP.safetyConfigs[-1].safetyParam & ToyotaSafetyFlags.EPS_PROBE)
    self.assertEqual(probe.CP.safetyConfigs[-1].safetyParam & 0xFF, 73)
    self.assertTrue(eps_probe_active(probe.CP))
    self.assertEqual(probe.CC.params.STEER_MAX, EPS_PROBE_STEER_MAX)
    self.assertEqual(EPS_PROBE_STEER_MAX, 3000)

  def test_angle_platform_not_eligible(self):
    CI = make_ci(CAR.TOYOTA_RAV4_TSS2_2023)
    self.assertEqual(CI.CP.steerControlType, structs.CarParams.SteerControlType.angle)
    self.assertIs(with_eps_probe(CI), CI)
    self.assertFalse(CI.CP.flags & ToyotaFlags.EPS_PROBE)
    self.assertFalse(CI.CP.safetyConfigs[-1].safetyParam & ToyotaSafetyFlags.EPS_PROBE)

  def test_other_brand_bit_not_probe(self):
    CI = make_ci(HYUNDAI.HYUNDAI_SONATA)
    CI.CP.flags |= int(ToyotaFlags.EPS_PROBE)
    self.assertFalse(eps_probe_active(CI.CP))
    self.assertIs(with_eps_probe(CI), CI)

  def test_controller_reaches_3000(self):
    self.assertEqual(final_torque(with_eps_probe(make_ci(CAR.TOYOTA_COROLLA_TSS2))), 3000)
    self.assertEqual(final_torque(make_ci(CAR.TOYOTA_COROLLA_TSS2)), 1500)


if __name__ == "__main__":
  unittest.main()
