import unittest

from opendbc.car import structs
from opendbc.can import CANPacker, CANParser
from opendbc.car.toyota.values import ToyotaFlags
from opendbc.sunnypilot.car.interfaces import _initialize_toyota
from opendbc.sunnypilot.car.toyota.auto_brake_hold import create_auto_brake_hold_command
from opendbc.sunnypilot.car.toyota.values import ToyotaFlagsSP, ToyotaSafetyFlagsSP


EXPECTED_SIGNALS = {
  "DSS1GDRV", "DS1STAT2", "DS1STBK2", "PCSWAR", "PCSALM",
  "PCSOPR", "PCSABK", "PBATRGR", "PPTRGR", "IBTRGR",
  "CLEXTRGR", "IRLT_REQ", "BRKHLD", "AVSTRGR", "VGRSTRGR",
  "PREFILL", "PBRTRGR", "PCSDIS", "PBPREPMP",
}


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


class TestAutoBrakeHoldPacking(unittest.TestCase):
  def setUp(self):
    self.packer = CANPacker("toyota_nodsu_pt_generated")

  def test_pre_collision_2_has_all_passthrough_signals(self):
    self.assertEqual(EXPECTED_SIGNALS | {"CHECKSUM"}, set(self.packer.dbc.name_to_msg["PRE_COLLISION_2"].sigs))

  def test_inactive_command_preserves_stock_signals(self):
    stock = {
      "DSS1GDRV": -0.5, "DS1STAT2": 3, "DS1STBK2": 2,
      "PCSWAR": 1, "PCSALM": 1, "PCSOPR": 1, "PCSABK": 1,
      "PBATRGR": 2, "PPTRGR": 1, "IBTRGR": 1, "CLEXTRGR": 1,
      "IRLT_REQ": 2, "BRKHLD": 1, "AVSTRGR": 1, "VGRSTRGR": 2,
      "PREFILL": 1, "PBRTRGR": 1, "PCSDIS": 1, "PBPREPMP": 1,
    }
    msg = create_auto_brake_hold_command(self.packer, 0, stock, False)
    parser = CANParser("toyota_nodsu_pt_generated", [("PRE_COLLISION_2", 33)], 0)
    parser.update([1, [msg]])
    for signal, expected in stock.items():
      self.assertEqual(expected, parser.vl["PRE_COLLISION_2"][signal])

  def test_active_command_sets_brake_hold_trigger_on_schedule(self):
    parser = CANParser("toyota_nodsu_pt_generated", [("PRE_COLLISION_2", 33)], 0)
    stock = {signal: 0 for signal in EXPECTED_SIGNALS}

    parser.update([1, [create_auto_brake_hold_command(self.packer, 0, stock, True)]])
    self.assertEqual(1, parser.vl["PRE_COLLISION_2"]["PBRTRGR"])

    parser.update([2, [create_auto_brake_hold_command(self.packer, 728, stock, True)]])
    self.assertEqual(0, parser.vl["PRE_COLLISION_2"]["PBRTRGR"])


if __name__ == "__main__":
  unittest.main()
