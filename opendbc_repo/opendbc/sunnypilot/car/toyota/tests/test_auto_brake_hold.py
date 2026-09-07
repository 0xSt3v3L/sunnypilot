import unittest
from dataclasses import dataclass, field

from opendbc.car import Bus, CanData, DT_CTRL, structs
from opendbc.can import CANPacker, CANParser
from opendbc.car.toyota.carstate import CarState
from opendbc.car.toyota.values import CAR, ToyotaFlags
from opendbc.sunnypilot.car.interfaces import _initialize_toyota
from opendbc.sunnypilot.car.toyota.auto_brake_hold import AutoBrakeHoldController, create_auto_brake_hold_command
from opendbc.sunnypilot.car.toyota.carstate_ext import CarStateExt
from opendbc.sunnypilot.car.toyota.values import ToyotaFlagsSP, ToyotaSafetyFlagsSP


EXPECTED_SIGNALS = {
  "DSS1GDRV", "DS1STAT2", "DS1STBK2", "PCSWAR", "PCSALM",
  "PCSOPR", "PCSABK", "PBATRGR", "PPTRGR", "IBTRGR",
  "CLEXTRGR", "IRLT_REQ", "BRKHLD", "AVSTRGR", "VGRSTRGR",
  "PREFILL", "PBRTRGR", "PCSDIS", "PBPREPMP",
}

GearShifter = structs.CarState.GearShifter


@dataclass
class CruiseState:
  available: bool = True
  enabled: bool = False


@dataclass
class Out:
  standstill: bool = True
  brakePressed: bool = False
  gasPressed: bool = False
  gearShifter: GearShifter = GearShifter.drive
  cruiseState: CruiseState = field(default_factory=CruiseState)


@dataclass
class State:
  out: Out = field(default_factory=Out)
  pre_collision_2: dict[str, float] = field(default_factory=dict)
  pre_collision_2_seen: bool = False
  pre_collision_2_ts_nanos: int = 0


class TestAutoBrakeHoldController(unittest.TestCase):
  def setUp(self):
    cp = structs.CarParams()
    cp_sp = structs.CarParamsSP()
    cp_sp.flags = ToyotaFlagsSP.AUTO_BRAKE_HOLD
    self.controller = AutoBrakeHoldController(cp, cp_sp)
    self.state = State()

  def activate(self):
    self.state.out.standstill = True
    self.state.out.brakePressed = True
    for _ in range(round(0.5 / DT_CTRL)):
      self.controller.update_state(self.state)
    self.assertTrue(self.controller.active)

  def test_timer_starts_only_after_standstill(self):
    self.state.out.brakePressed = True
    self.state.out.standstill = False
    for _ in range(100):
      self.controller.update_state(self.state)
    self.assertFalse(self.controller.active)
    self.assertEqual(0, self.controller.activation_frames)

  def test_activates_after_500_ms_at_standstill(self):
    self.state.out.brakePressed = True
    self.state.out.standstill = True
    for _ in range(round(0.5 / DT_CTRL) - 1):
      self.controller.update_state(self.state)
    self.assertFalse(self.controller.active)
    self.controller.update_state(self.state)
    self.assertTrue(self.controller.active)

  def test_brake_release_before_activation_resets_timer(self):
    self.state.out.brakePressed = True
    self.state.out.standstill = True
    for _ in range(25):
      self.controller.update_state(self.state)
    self.state.out.brakePressed = False
    self.controller.update_state(self.state)
    self.assertEqual(0, self.controller.activation_frames)
    self.assertFalse(self.controller.active)

  def test_brake_release_after_activation_keeps_active(self):
    self.activate()
    self.state.out.brakePressed = False
    self.controller.update_state(self.state)
    self.assertTrue(self.controller.active)

  def test_second_brake_press_keeps_active(self):
    self.activate()
    self.state.out.brakePressed = False
    self.controller.update_state(self.state)
    self.state.out.brakePressed = True
    self.controller.update_state(self.state)
    self.assertTrue(self.controller.active)

  def test_release_conditions_clear_active_state_immediately(self):
    cases = (
      ("gas", lambda state: setattr(state.out, "gasPressed", True)),
      ("movement", lambda state: setattr(state.out, "standstill", False)),
      ("ACC enable", lambda state: setattr(state.out.cruiseState, "enabled", True)),
      ("cruise-main off", lambda state: setattr(state.out.cruiseState, "available", False)),
      ("park", lambda state: setattr(state.out, "gearShifter", GearShifter.park)),
      ("reverse", lambda state: setattr(state.out, "gearShifter", GearShifter.reverse)),
    )
    for name, mutate in cases:
      with self.subTest(name=name):
        self.setUp()
        self.activate()
        mutate(self.state)
        self.controller.update_state(self.state)
        self.assertFalse(self.controller.active)
        self.assertEqual(0, self.controller.activation_frames)

  def test_update_requires_seen_source_data(self):
    self.activate()
    packer = CANPacker("toyota_nodsu_pt_generated")
    self.state.pre_collision_2 = {signal: 0 for signal in EXPECTED_SIGNALS}
    self.state.pre_collision_2_ts_nanos = 1_000_000_000
    self.assertEqual([], self.controller.update(self.state, packer, 0, 1_000_000_000))

  def test_update_rejects_source_older_than_100_ms(self):
    self.activate()
    packer = CANPacker("toyota_nodsu_pt_generated")
    self.state.pre_collision_2 = {signal: 0 for signal in EXPECTED_SIGNALS}
    self.state.pre_collision_2_seen = True
    self.state.pre_collision_2_ts_nanos = 1_000_000_000
    self.assertEqual([], self.controller.update(self.state, packer, 0, 1_100_000_001))

  def test_update_sends_fresh_source_data_on_even_frames(self):
    self.activate()
    packer = CANPacker("toyota_nodsu_pt_generated")
    self.state.pre_collision_2 = {signal: 0 for signal in EXPECTED_SIGNALS}
    self.state.pre_collision_2_seen = True
    self.state.pre_collision_2_ts_nanos = 1_000_000_000
    self.assertEqual(1, len(self.controller.update(self.state, packer, 0, 1_100_000_000)))

  def test_update_skips_odd_frames(self):
    self.activate()
    packer = CANPacker("toyota_nodsu_pt_generated")
    self.state.pre_collision_2 = {signal: 0 for signal in EXPECTED_SIGNALS}
    self.state.pre_collision_2_seen = True
    self.state.pre_collision_2_ts_nanos = 1_000_000_000
    self.assertEqual([], self.controller.update(self.state, packer, 1, 1_000_000_000))


class TestAutoBrakeHoldCarState(unittest.TestCase):
  def make_cp(self, enabled: bool):
    cp = structs.CarParams()
    cp.carFingerprint = CAR.TOYOTA_RAV4_TSS2
    cp_sp = structs.CarParamsSP()
    cp_sp.flags = ToyotaFlagsSP.AUTO_BRAKE_HOLD if enabled else 0
    return cp, cp_sp

  def test_camera_parser_subscribes_only_when_enabled(self):
    cp, cp_sp = self.make_cp(True)
    enabled_parser = CarState.get_can_parsers(cp, cp_sp)[Bus.cam]
    self.assertIn("PRE_COLLISION_2", enabled_parser.vl)

    cp, cp_sp = self.make_cp(False)
    disabled_parser = CarState.get_can_parsers(cp, cp_sp)[Bus.cam]
    self.assertNotIn("PRE_COLLISION_2", disabled_parser.vl)

  def test_new_camera_message_updates_values_and_timestamp(self):
    cp, cp_sp = self.make_cp(True)
    state_ext = CarStateExt(cp, cp_sp)
    self.assertEqual({}, state_ext.pre_collision_2)
    self.assertFalse(state_ext.pre_collision_2_seen)
    self.assertEqual(0, state_ext.pre_collision_2_ts_nanos)

    pt_parser = CANParser("toyota_nodsu_pt_generated", [], 0)
    cam_parser = CANParser("toyota_nodsu_pt_generated", [("PRE_COLLISION_2", 33)], 2)
    packer = CANPacker("toyota_nodsu_pt_generated")
    stock = {signal: 0 for signal in EXPECTED_SIGNALS}
    stock["DS1STAT2"] = 3
    msg = create_auto_brake_hold_command(packer, 0, stock, False)
    timestamp = 1_234_567_890
    cam_parser.update([timestamp, [CanData(msg[0], msg[1], 2)]])

    state_ext.update(structs.CarState(), structs.CarStateSP(), {Bus.pt: pt_parser, Bus.cam: cam_parser})
    self.assertTrue(state_ext.pre_collision_2_seen)
    self.assertEqual(3, state_ext.pre_collision_2["DS1STAT2"])
    self.assertEqual(timestamp, state_ext.pre_collision_2_ts_nanos)


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
