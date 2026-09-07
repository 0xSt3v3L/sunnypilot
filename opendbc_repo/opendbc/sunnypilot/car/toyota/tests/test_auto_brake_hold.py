import unittest

from opendbc.car import structs
from opendbc.car.toyota.values import ToyotaFlags
from opendbc.sunnypilot.car.interfaces import _initialize_toyota
from opendbc.sunnypilot.car.toyota.values import ToyotaFlagsSP, ToyotaSafetyFlagsSP


class TestAutoBrakeHoldEligibility(unittest.TestCase):
  def make_cp(self, flags=ToyotaFlags.TSS2):
    cp = structs.CarParams()
    cp.brand = "toyota"
    cp.flags = int(flags)
    cp.openpilotLongitudinalControl = True
    cp.safetyConfigs = [structs.CarParams.SafetyConfig()]
    return cp

  def test_camera_acc_tss2_is_enabled(self):
    cp, cp_sp = self.make_cp(), structs.CarParamsSP()
    _initialize_toyota(cp, cp_sp, {"ToyotaAutoHold": "1"})
    self.assertTrue(cp_sp.flags & ToyotaFlagsSP.AUTO_BRAKE_HOLD)
    self.assertTrue(cp_sp.safetyParam & ToyotaSafetyFlagsSP.AUTO_BRAKE_HOLD)

  def test_incompatible_configurations_are_rejected(self):
    cases = (
      (ToyotaFlags.TSS2 | ToyotaFlags.RADAR_ACC, True),
      (ToyotaFlags.TSS2 | ToyotaFlags.SECOC, True),
      (ToyotaFlags.TSS2, False),
    )
    for flags, openpilot_long in cases:
      cp, cp_sp = self.make_cp(flags), structs.CarParamsSP()
      cp.openpilotLongitudinalControl = openpilot_long
      _initialize_toyota(cp, cp_sp, {"ToyotaAutoHold": "1"})
      self.assertFalse(cp_sp.flags & ToyotaFlagsSP.AUTO_BRAKE_HOLD)
      self.assertFalse(cp_sp.safetyParam & ToyotaSafetyFlagsSP.AUTO_BRAKE_HOLD)

  def test_disabled_parameter_is_rejected(self):
    cp, cp_sp = self.make_cp(), structs.CarParamsSP()
    _initialize_toyota(cp, cp_sp, {"ToyotaAutoHold": "0"})
    self.assertFalse(cp_sp.flags & ToyotaFlagsSP.AUTO_BRAKE_HOLD)


if __name__ == "__main__":
  unittest.main()
