#!/usr/bin/env python3
import numpy as np
import random
import unittest
import itertools

from opendbc.car.toyota.values import ToyotaSafetyFlags
from opendbc.car.structs import CarParams
from opendbc.safety import ALTERNATIVE_EXPERIENCE
from opendbc.safety.tests.libsafety import libsafety_py
import opendbc.safety.tests.common as common

TOYOTA_COMMON_TX_MSGS = [[0x2E4, 0], [0x191, 0], [0x412, 0], [0x343, 0], [0x1D2, 0]]  # LKAS + LTA + ACC & PCM cancel cmds
TOYOTA_SECOC_TX_MSGS = [[0x131, 0], [0x183, 0]] + TOYOTA_COMMON_TX_MSGS
TOYOTA_COMMON_LONG_TX_MSGS = [[0x283, 0], [0x2E6, 0], [0x2E7, 0], [0x33E, 0], [0x344, 0], [0x365, 0], [0x366, 0], [0x4CB, 0],  # DSU bus 0
                              [0x128, 1], [0x141, 1], [0x160, 1], [0x161, 1], [0x470, 1],  # DSU bus 1
                              [0x411, 0],  # PCS_HUD
                              [0x750, 0]]  # radar diagnostic address


class TestToyotaSafetyBase(common.CarSafetyTest, common.LongitudinalAccelSafetyTest):

  DBC = "toyota_nodsu_pt_generated"
  SAFETY_MODEL = CarParams.SafetyModel.toyota

  TX_MSGS = TOYOTA_COMMON_TX_MSGS + TOYOTA_COMMON_LONG_TX_MSGS
  RELAY_MALFUNCTION_ADDRS = {0: (0x2E4, 0x191, 0x412, 0x343)}
  FWD_BLACKLISTED_ADDRS = {2: [0x2E4, 0x412, 0x191, 0x343]}
  EPS_SCALE = 73

  def _torque_meas_msg(self, torque: int, driver_torque: int | None = None):
    values = {"STEER_TORQUE_EPS": (torque / self.EPS_SCALE) * 100.}
    if driver_torque is not None:
      values["STEER_TORQUE_DRIVER"] = driver_torque
    return self.packer.make_can_msg_safety("STEER_TORQUE_SENSOR", 0, values)

  # Both torque and angle safety modes test with each other's steering commands
  def _torque_cmd_msg(self, torque, steer_req=1):
    values = {"STEER_TORQUE_CMD": torque, "STEER_REQUEST": steer_req}
    return self.packer.make_can_msg_safety("STEERING_LKA", 0, values)

  def _angle_meas_msg(self, angle: float, steer_angle_initializing: bool = False):
    # This creates a steering torque angle message. Not set on all platforms,
    # relative to init angle on some older TSS2 platforms. Only to be used with LTA
    values = {"STEER_ANGLE": angle, "STEER_ANGLE_INITIALIZING": int(steer_angle_initializing)}
    return self.packer.make_can_msg_safety("STEER_TORQUE_SENSOR", 0, values)

  def _angle_cmd_msg(self, angle: float, enabled: bool):
    return self._lta_msg(int(enabled), int(enabled), angle, torque_wind_down=100 if enabled else 0)

  def _lta_msg(self, req, req2, angle_cmd, torque_wind_down=100):
    values = {"STEER_REQUEST": req, "STEER_REQUEST_2": req2, "STEER_ANGLE_CMD": angle_cmd, "TORQUE_WIND_DOWN": torque_wind_down}
    return self.packer.make_can_msg_safety("STEERING_LTA", 0, values)

  def _accel_msg_343(self, accel, cancel_req=0):
    values = {"ACCEL_CMD": accel, "CANCEL_REQ": cancel_req}
    return self.packer.make_can_msg_safety("ACC_CONTROL", 0, values)

  def _accel_msg(self, accel, cancel_req=0):
    return self._accel_msg_343(accel, cancel_req)

  def _speed_msg(self, speed, quality_flag=True):
    values = {("WHEEL_SPEED_%s" % n): speed * 3.6 for n in ["FR", "FL", "RR", "RL"]}
    if not quality_flag:
      values |= {"WHEEL_SPEED_%s_FAULT" % n: 1.0 for n in ["FR", "FL", "RR", "RL"]}
    return self.packer.make_can_msg_safety("WHEEL_SPEEDS", 0, values)

  def _user_brake_msg(self, brake):
    values = {"BRAKE_PRESSED": brake}
    return self.packer.make_can_msg_safety("BRAKE_MODULE", 0, values)

  def _user_gas_msg(self, gas):
    cruise_active = self.safety.get_controls_allowed()
    values = {"GAS_RELEASED": not gas, "CRUISE_ACTIVE": cruise_active}
    return self.packer.make_can_msg_safety("PCM_CRUISE", 0, values)

  def _pcm_status_msg(self, enable):
    values = {"CRUISE_ACTIVE": enable}
    return self.packer.make_can_msg_safety("PCM_CRUISE", 0, values)

  def _acc_main_msg(self, main_on: bool):
    values = {"MAIN_ON": main_on}
    return self.packer.make_can_msg_safety("PCM_CRUISE_2", 0, values)

  def test_diagnostics(self, stock_longitudinal: bool = False, ecu_disabled: bool = True):
    for should_tx, msg in ((False, b"\x6D\x02\x3E\x00\x00\x00\x00\x00"),  # fwdCamera tester present
                           (False, b"\x0F\x03\xAA\xAA\x00\x00\x00\x00"),  # non-tester present
                           (True, b"\x0F\x02\x3E\x00\x00\x00\x00\x00")):
      tester_present = libsafety_py.make_CANPacket(0x750, 0, msg)
      self.assertEqual(should_tx and ecu_disabled and not stock_longitudinal, self._tx(tester_present))

  def test_block_aeb(self, stock_longitudinal: bool = False):
    for controls_allowed in (True, False):
      for bad in (True, False):
        for _ in range(10):
          self.safety.set_controls_allowed(controls_allowed)
          dat = [random.randint(1, 255) for _ in range(7)]
          if not bad:
            dat = [0]*6 + dat[-1:]
          msg = libsafety_py.make_CANPacket(0x283, 0, bytes(dat))
          self.assertEqual(not bad and not stock_longitudinal, self._tx(msg))

  # Only allow LTA msgs with no actuation
  def test_lta_steer_cmd(self):
    for engaged, req, req2, torque_wind_down, angle in itertools.product([True, False],
                                                                  [0, 1], [0, 1],
                                                                  [0, 50, 100],
                                                                  np.linspace(-20, 20, 5)):
      self.safety.set_controls_allowed(engaged)

      should_tx = not req and not req2 and angle == 0 and torque_wind_down == 0
      self.assertEqual(should_tx, self._tx(self._lta_msg(req, req2, angle, torque_wind_down)),
                       f"{req=} {req2=} {angle=} {torque_wind_down=}")

  def test_rx_hook(self):
    # checksum checks
    for msg in ["trq", "pcm"]:
      self.safety.set_controls_allowed(1)
      if msg == "trq":
        msg = self._torque_meas_msg(0)
      if msg == "pcm":
        msg = self._pcm_status_msg(True)
      self.assertTrue(self._rx(msg))
      msg[0].data[4] = 0
      msg[0].data[5] = 0
      msg[0].data[6] = 0
      msg[0].data[7] = 0
      self.assertFalse(self._rx(msg))
      self.assertFalse(self.safety.get_controls_allowed())

    # quality flag tests
    msg = self._speed_msg(0)
    self.assertTrue(self._rx(msg))

    msg = self._speed_msg(0, quality_flag=False)
    self.assertFalse(self._rx(msg))

  def test_vehicle_speed_measurements(self):
    # OVERRIDDEN: 72.22_ is the max speed in m/s
    self._common_measurement_test(self._speed_msg, 0, 259 / 3.6, 1,
                                  self.safety.get_vehicle_speed_min, self.safety.get_vehicle_speed_max)

  # Route 000000a6: RES while braking at 6 km/h. CRUISE_ACTIVE 0->1 arrived just before BRAKE_PRESSED 1->0, the stale
  # brake revoked controls, the PCM never got ACC_CONTROL and latched a cruise fault until the next ignition cycle.
  def _engage_while_braking(self, moving=True):
    self._rx(self._vehicle_moving_msg(self.STANDSTILL_THRESHOLD + 1 if moving else 0))
    self._rx(self._pcm_status_msg(False))
    self._rx(self._user_brake_msg(1))
    self._rx(self._pcm_status_msg(True))

  def test_engage_with_brake_release_reported_late(self):
    self._engage_while_braking()
    self.assertFalse(self.safety.get_controls_allowed())
    self._rx(self._user_brake_msg(0))
    self.assertTrue(self.safety.get_controls_allowed())
    # the test PCM_CRUISE message reports gas pressed; with gas released ACC_CONTROL can be sent
    self._rx(self._user_gas_msg(0))
    self.assertTrue(self.safety.get_longitudinal_allowed())

  def test_engage_with_brake_held_while_moving_is_not_allowed(self):
    self._engage_while_braking()
    self._rx(self._user_brake_msg(1))
    self.assertFalse(self.safety.get_controls_allowed())
    # releasing it afterwards needs a new cruise rising edge
    self._rx(self._user_brake_msg(0))
    self._rx(self._pcm_status_msg(True))
    self.assertFalse(self.safety.get_controls_allowed())

  def test_next_cruise_message_before_the_brake_message_drops_the_edge(self):
    self._engage_while_braking()
    self._rx(self._pcm_status_msg(True))
    self._rx(self._user_brake_msg(0))
    self.assertFalse(self.safety.get_controls_allowed())

  def test_engage_waits_for_the_next_brake_message_not_a_time(self):
    # no timer: the next PCM_CRUISE message bounds how late the brake message can be
    self.safety.set_timer(1000000)
    self._engage_while_braking()
    self.safety.set_timer(1000000 + 500000)
    self._rx(self._vehicle_moving_msg(self.STANDSTILL_THRESHOLD + 1))
    self.assertFalse(self.safety.get_controls_allowed())
    self._rx(self._user_brake_msg(0))
    self.assertTrue(self.safety.get_controls_allowed())

  def test_only_the_first_brake_message_after_the_edge_counts(self):
    self._engage_while_braking()
    self._rx(self._user_brake_msg(1))
    self._rx(self._user_brake_msg(0))
    self.assertFalse(self.safety.get_controls_allowed())

  def test_cruise_off_before_brake_release_is_not_allowed(self):
    self._engage_while_braking()
    self._rx(self._pcm_status_msg(False))
    self._rx(self._user_brake_msg(0))
    self.assertFalse(self.safety.get_controls_allowed())

  def test_brake_after_engagement_does_not_regrant(self):
    self._rx(self._vehicle_moving_msg(self.STANDSTILL_THRESHOLD + 1))
    self._rx(self._user_brake_msg(0))
    self._rx(self._pcm_status_msg(False))
    self._rx(self._pcm_status_msg(True))
    self.assertTrue(self.safety.get_controls_allowed())
    self._rx(self._user_brake_msg(1))
    self.assertFalse(self.safety.get_controls_allowed())
    self._rx(self._pcm_status_msg(True))
    self._rx(self._user_brake_msg(0))
    self.assertFalse(self.safety.get_controls_allowed())

  def test_engage_at_standstill_with_brake_held_is_unchanged(self):
    self._engage_while_braking(moving=False)
    self.assertTrue(self.safety.get_controls_allowed())


class TestToyotaSafetyTorque(TestToyotaSafetyBase, common.MotorTorqueSteeringSafetyTest, common.SteerRequestCutSafetyTest):

  MAX_RATE_UP = 15
  MAX_RATE_DOWN = 25
  MAX_TORQUE_LOOKUP = [0], [1500]
  MAX_RT_DELTA = 450
  MAX_TORQUE_ERROR = 350
  TORQUE_MEAS_TOLERANCE = 1  # toyota safety adds one to be conservative for rounding

  # Safety around steering req bit
  MIN_VALID_STEERING_FRAMES = 17
  MAX_INVALID_STEERING_FRAMES = 1

  @property
  def SAFETY_PARAM(self):
    return self.EPS_SCALE


class TestToyotaMadsSafety(TestToyotaSafetyTorque):
  """MADS (ALT_EXP_ENABLE_MADS): lateral-only engagement, hard-coded for the Corolla TSS2 config.

  Inherited torque tests run with MADS off, as a regression check that nothing changes without the flag.
  Each MADS test enables the flag itself.
  """

  def test_acc_main_on_parsed_from_pcm_cruise_2(self):
    for main_on in (True, False, True):
      self.assertTrue(self._rx(self._acc_main_msg(main_on)))
      self.assertEqual(self.safety.get_acc_main_on(), main_on)

  def test_pcm_cruise_2_is_a_required_rx_message(self):
    self.safety.set_timer(int(2e6))
    for msg in (self._speed_msg(0), self._torque_meas_msg(0), self._pcm_status_msg(False), self._user_brake_msg(False)):
      self.assertTrue(self._rx(msg))
    self.safety.safety_tick()
    self.assertFalse(self.safety.safety_config_valid())

    self.assertTrue(self._rx(self._acc_main_msg(True)))
    self.safety.safety_tick()
    self.assertTrue(self.safety.safety_config_valid())

  def _enable_mads(self):
    self.safety.set_alternative_experience(ALTERNATIVE_EXPERIENCE.ENABLE_MADS)

  def _grant_lateral_via_main(self):
    self._enable_mads()
    self.assertTrue(self._rx(self._acc_main_msg(False)))
    self.assertTrue(self._rx(self._acc_main_msg(True)))
    self.assertTrue(self.safety.get_controls_allowed_lateral())

  def test_mads_disabled_never_grants_lateral(self):
    # only ALT_EXP_ENABLE_MADS may enable MADS, no other alternative experience bit
    all_other_bits = 0xFFFF & ~ALTERNATIVE_EXPERIENCE.ENABLE_MADS
    for alt_exp in (ALTERNATIVE_EXPERIENCE.DEFAULT, all_other_bits):
      with self.subTest(alt_exp=alt_exp):
        self.safety.set_alternative_experience(alt_exp)
        self._reset_safety_hooks()
        self.assertTrue(self._rx(self._acc_main_msg(False)))
        self.assertTrue(self._rx(self._acc_main_msg(True)))
        self.assertTrue(self._rx(self._pcm_status_msg(False)))
        self.assertTrue(self._rx(self._pcm_status_msg(True)))
        self.assertTrue(self.safety.get_controls_allowed())
        self.assertFalse(self.safety.get_controls_allowed_lateral())

  def test_acc_main_rising_edge_grants_lateral_only(self):
    self._grant_lateral_via_main()
    self.assertFalse(self.safety.get_controls_allowed())

  def test_acc_main_falling_edge_revokes_lateral(self):
    self._grant_lateral_via_main()
    self.assertTrue(self._rx(self._acc_main_msg(False)))
    self.assertFalse(self.safety.get_controls_allowed_lateral())

  def test_lateral_requires_a_new_edge(self):
    self._grant_lateral_via_main()
    self.safety.set_controls_allowed_lateral(False)
    for _ in range(3):
      self.assertTrue(self._rx(self._acc_main_msg(True)))
    self.assertFalse(self.safety.get_controls_allowed_lateral())

  def test_cruise_engage_grants_lateral(self):
    self._grant_lateral_via_main()
    self.safety.set_controls_allowed_lateral(False)
    self._rx(self._pcm_status_msg(False))
    self._rx(self._pcm_status_msg(True))
    self.assertTrue(self.safety.get_controls_allowed())
    self.assertTrue(self.safety.get_controls_allowed_lateral())

  def test_cruise_engage_without_acc_main_does_not_grant_lateral(self):
    # lateral permission implies ACC main is on: a controls_allowed rising edge with MAIN off must not grant it
    self._enable_mads()
    self.assertTrue(self._rx(self._acc_main_msg(False)))
    self.assertTrue(self._rx(self._pcm_status_msg(False)))
    self.assertTrue(self._rx(self._pcm_status_msg(True)))
    self.assertTrue(self.safety.get_controls_allowed())
    self.assertFalse(self.safety.get_controls_allowed_lateral())

  def test_reinit_with_acc_main_on_regrants_lateral(self):
    # MAIN still on after a safety mode re-init must grant again, so mads_reset has to clear the previous MAIN state
    self._grant_lateral_via_main()
    self._reset_safety_hooks()
    self.assertFalse(self.safety.get_controls_allowed_lateral())
    self.assertTrue(self._rx(self._acc_main_msg(True)))
    self.assertTrue(self.safety.get_controls_allowed_lateral())

  def test_brake_keeps_lateral(self):
    self._grant_lateral_via_main()
    self._rx(self._pcm_status_msg(True))
    self.assertTrue(self.safety.get_controls_allowed())
    self._rx(self._speed_msg(10))
    self._rx(self._user_brake_msg(True))
    self.assertFalse(self.safety.get_controls_allowed())
    self.assertTrue(self.safety.get_controls_allowed_lateral())

  def test_gas_keeps_lateral(self):
    self._grant_lateral_via_main()
    self._rx(self._user_gas_msg(1))
    self.assertTrue(self.safety.get_controls_allowed_lateral())

  def test_invalid_rx_message_revokes_lateral(self):
    self._grant_lateral_via_main()
    msg = self._acc_main_msg(True)
    msg[0].data[7] = (msg[0].data[7] + 1) % 256  # corrupt the checksum byte
    self.assertFalse(self._rx(msg))
    self.assertFalse(self.safety.get_controls_allowed_lateral())

  def test_safety_tick_revokes_lateral(self):
    self._grant_lateral_via_main()
    self.safety.set_timer(int(2e6))
    self.safety.safety_tick()
    self.assertFalse(self.safety.get_controls_allowed_lateral())

  def test_safety_mode_change_resets_lateral(self):
    self._grant_lateral_via_main()
    self._reset_safety_hooks()
    self.assertFalse(self.safety.get_controls_allowed_lateral())

  def test_heartbeat_mismatch_revokes_lateral(self):
    self._grant_lateral_via_main()
    self.safety.set_heartbeat_engaged_mads(False)
    for _ in range(3):
      self.assertTrue(self.safety.get_controls_allowed_lateral())
      self.safety.mads_heartbeat_engaged_check()
    self.assertFalse(self.safety.get_controls_allowed_lateral())

  def test_heartbeat_engaged_keeps_lateral(self):
    self._grant_lateral_via_main()
    self.safety.set_heartbeat_engaged_mads(True)
    for _ in range(10):
      self.safety.mads_heartbeat_engaged_check()
    self.assertTrue(self.safety.get_controls_allowed_lateral())

  def test_main_on_at_safety_init_grants_then_heartbeat_revokes(self):
    # first PCM_CRUISE_2 after a safety mode init already reports MAIN_ON
    self._enable_mads()
    self._reset_safety_hooks()
    self.assertTrue(self._rx(self._acc_main_msg(True)))
    self.assertTrue(self.safety.get_controls_allowed_lateral())
    self.safety.set_heartbeat_engaged_mads(False)
    for _ in range(3):
      self.safety.mads_heartbeat_engaged_check()
    self.assertFalse(self.safety.get_controls_allowed_lateral())

  def _set_prev_torque(self, t):
    self.safety.set_desired_torque_last(t)
    self.safety.set_rt_torque_last(t)
    self.safety.set_torque_meas(t, t)

  def test_lateral_only_allows_steering_within_limits(self):
    self._grant_lateral_via_main()
    self.assertFalse(self.safety.get_controls_allowed())
    self._set_prev_torque(0)
    self.assertTrue(self._tx(self._torque_cmd_msg(self.MAX_RATE_UP)))

  def test_no_lateral_blocks_steering(self):
    self._set_prev_torque(0)
    self.assertFalse(self.safety.get_controls_allowed_lateral())
    self.assertFalse(self._tx(self._torque_cmd_msg(self.MAX_RATE_UP)))

  def test_lateral_only_keeps_torque_limits(self):
    self._grant_lateral_via_main()
    # rate limit
    self._set_prev_torque(0)
    self.assertFalse(self._tx(self._torque_cmd_msg(self.MAX_RATE_UP + 1)))
    # torque without the steer request bit
    self._set_prev_torque(0)
    self.assertFalse(self._tx(self._torque_cmd_msg(self.MAX_RATE_UP, steer_req=0)))
    # global max torque
    max_torque = self.MAX_TORQUE_LOOKUP[1][-1]
    self._set_prev_torque(max_torque)
    self.assertFalse(self._tx(self._torque_cmd_msg(max_torque + 1)))

  def test_lateral_only_blocks_longitudinal(self):
    self._grant_lateral_via_main()
    self.assertFalse(self.safety.get_controls_allowed())
    self.assertFalse(self._tx(self._accel_msg(1.0)))

  def test_relay_malfunction_blocks_lateral_only_steering(self):
    self._grant_lateral_via_main()
    self._set_prev_torque(0)
    self.safety.set_relay_malfunction(True)
    self.assertFalse(self._tx(self._torque_cmd_msg(self.MAX_RATE_UP)))

  def test_mads_only_on_toyota_safety_mode(self):
    # allOutput init sets controls_allowed, so the next rx is a controls_allowed rising edge
    self.safety.set_safety_hooks(CarParams.SafetyModel.allOutput, 0)
    self._enable_mads()
    self._rx(self._speed_msg(0))
    self.assertTrue(self.safety.get_controls_allowed())
    self.assertFalse(self.safety.get_controls_allowed_lateral())


class _LateralOnlyPermission:
  """Safety proxy that maps controls_allowed to lateral-only permission (controls_allowed stays False)."""

  def __init__(self, safety):
    self._safety = safety

  def __getattr__(self, name):
    return getattr(self._safety, name)

  def set_controls_allowed(self, c):
    self._safety.set_controls_allowed(False)
    self._safety.set_controls_allowed_lateral(bool(c))

  def get_controls_allowed(self):
    return self._safety.get_controls_allowed_lateral()


class TestToyotaMadsLateralOnlyTorque(TestToyotaMadsSafety):
  """Re-runs the torque steering tests with lateral-only permission, which must be held to the same limits."""

  def setUp(self):
    super().setUp()
    self.safety.set_alternative_experience(ALTERNATIVE_EXPERIENCE.ENABLE_MADS)
    self.safety = _LateralOnlyPermission(self.safety)


_TORQUE_TESTS = {name for base in (common.TorqueSteeringSafetyTestBase, common.SteerRequestCutSafetyTest,
                                   common.MotorTorqueSteeringSafetyTest)
                 for name in vars(base) if name.startswith("test_")}
for _name in dir(TestToyotaMadsLateralOnlyTorque):
  if _name.startswith("test_") and _name not in _TORQUE_TESTS:
    setattr(TestToyotaMadsLateralOnlyTorque, _name, None)  # not collected by unittest


class TestToyotaSafetyAngle(TestToyotaSafetyBase, common.AngleSteeringSafetyTest):

  # Angle control limits
  STEER_ANGLE_MAX = 94.9461  # deg
  DEG_TO_CAN = 17.452007  # 1 / 0.0573 deg to can

  ANGLE_RATE_BP = [5., 25., 25.]
  ANGLE_RATE_UP = [0.3, 0.15, 0.15]  # windup limit
  ANGLE_RATE_DOWN = [0.36, 0.26, 0.26]  # unwind limit

  MAX_LTA_ANGLE = 94.9461  # PCS faults if commanding above this, deg
  MAX_MEAS_TORQUE = 1500  # max allowed measured EPS torque before wind down
  MAX_LTA_DRIVER_TORQUE = 150  # max allowed driver torque before wind down

  @property
  def SAFETY_PARAM(self):
    return self.EPS_SCALE | ToyotaSafetyFlags.LTA

  # Only allow LKA msgs with no actuation
  def test_lka_steer_cmd(self):
    for engaged, steer_req, torque in itertools.product([True, False],
                                                        [0, 1],
                                                        np.linspace(-1500, 1500, 7)):
      self.safety.set_controls_allowed(engaged)
      torque = int(torque)
      self.safety.set_rt_torque_last(torque)
      self.safety.set_torque_meas(torque, torque)
      self.safety.set_desired_torque_last(torque)

      should_tx = not steer_req and torque == 0
      self.assertEqual(should_tx, self._tx(self._torque_cmd_msg(torque, steer_req)))

  def test_lta_steer_cmd(self):
    """
    Tests the LTA steering command message
    controls_allowed:
    * STEER_REQUEST and STEER_REQUEST_2 do not mismatch
    * TORQUE_WIND_DOWN is only set to 0 or 100 when STEER_REQUEST and STEER_REQUEST_2 are both 1
    * Full torque messages are blocked if either EPS torque or driver torque is above the threshold

    not controls_allowed:
    * STEER_REQUEST, STEER_REQUEST_2, and TORQUE_WIND_DOWN are all 0
    """
    for controls_allowed in (True, False):
      for angle in np.arange(-90, 90, 1):
        self.safety.set_controls_allowed(controls_allowed)
        self._reset_angle_measurement(angle)
        self._set_prev_desired_angle(angle)

        self.assertTrue(self._tx(self._lta_msg(0, 0, angle, 0)))
        if controls_allowed:
          # Test the two steer request bits and TORQUE_WIND_DOWN torque wind down signal
          for req, req2, torque_wind_down in itertools.product([0, 1], [0, 1], [0, 50, 100]):
            mismatch = not (req or req2) and torque_wind_down != 0
            should_tx = req == req2 and (torque_wind_down in (0, 100)) and not mismatch
            self.assertEqual(should_tx, self._tx(self._lta_msg(req, req2, angle, torque_wind_down)))

        else:
          # Controls not allowed
          for req, req2, torque_wind_down in itertools.product([0, 1], [0, 1], [0, 50, 100]):
            should_tx = not (req or req2) and torque_wind_down == 0
            self.assertEqual(should_tx, self._tx(self._lta_msg(req, req2, angle, torque_wind_down)))

    # Test max EPS torque and driver override thresholds (independent of angle, test a few representative angles)
    for angle in (-89, 0, 89):
      self.safety.set_controls_allowed(True)
      self._reset_angle_measurement(angle)
      self._set_prev_desired_angle(angle)

      cases = itertools.product(
        (0, self.MAX_MEAS_TORQUE - 1, self.MAX_MEAS_TORQUE, self.MAX_MEAS_TORQUE + 1, self.MAX_MEAS_TORQUE * 2),
        (0, self.MAX_LTA_DRIVER_TORQUE - 1, self.MAX_LTA_DRIVER_TORQUE, self.MAX_LTA_DRIVER_TORQUE + 1, self.MAX_LTA_DRIVER_TORQUE * 2)
      )

      for eps_torque, driver_torque in cases:
        for sign in (-1, 1):
          for _ in range(6):
            self._rx(self._torque_meas_msg(sign * eps_torque, sign * driver_torque))

          # Toyota adds 1 to EPS torque since it is rounded after EPS factor
          should_tx = (eps_torque - 1) <= self.MAX_MEAS_TORQUE and driver_torque <= self.MAX_LTA_DRIVER_TORQUE
          self.assertEqual(should_tx, self._tx(self._lta_msg(1, 1, angle, 100)))
          self.assertTrue(self._tx(self._lta_msg(1, 1, angle, 0)))  # should tx if we wind down torque

  def test_angle_measurements(self):
    """
    * Tests angle meas quality flag dictates whether angle measurement is parsed, and if rx is valid
    * Tests rx hook correctly clips the angle measurement, since it is to be compared to LTA cmd when inactive
    """
    for steer_angle_initializing in (True, False):
      for angle in np.arange(0, self.STEER_ANGLE_MAX * 2, 1):
        # If init flag is set, do not rx or parse any angle measurements
        for a in (angle, -angle, 0, 0, 0, 0):
          self.assertEqual(not steer_angle_initializing,
                           self._rx(self._angle_meas_msg(a, steer_angle_initializing)))

        final_angle = 0 if steer_angle_initializing else round(angle * self.DEG_TO_CAN)
        self.assertEqual(self.safety.get_angle_meas_min(), -final_angle)
        self.assertEqual(self.safety.get_angle_meas_max(), final_angle)

        self._rx(self._angle_meas_msg(0))
        self.assertEqual(self.safety.get_angle_meas_min(), -final_angle)
        self.assertEqual(self.safety.get_angle_meas_max(), 0)

        self._rx(self._angle_meas_msg(0))
        self.assertEqual(self.safety.get_angle_meas_min(), 0)
        self.assertEqual(self.safety.get_angle_meas_max(), 0)


class TestToyotaEpsProbeSafety(TestToyotaSafetyTorque):
  """EPS probe mode (debug builds only): every torque check at the raised 3000 limit."""

  MAX_TORQUE_LOOKUP = [0], [3000]

  @property
  def SAFETY_PARAM(self):
    return self.EPS_SCALE | ToyotaSafetyFlags.EPS_PROBE


class TestToyotaAltBrakeSafety(TestToyotaSafetyTorque):

  DBC = "toyota_new_mc_pt_generated"

  @property
  def SAFETY_PARAM(self):
    return self.EPS_SCALE | ToyotaSafetyFlags.ALT_BRAKE

  def _user_brake_msg(self, brake):
    values = {"BRAKE_PRESSED": brake}
    return self.packer.make_can_msg_safety("BRAKE_MODULE", 0, values)

  # No LTA message in the DBC
  def test_lta_steer_cmd(self):
    pass


class TestToyotaStockLongitudinalBase(TestToyotaSafetyBase):

  TX_MSGS = TOYOTA_COMMON_TX_MSGS
  # Base addresses minus ACC_CONTROL (0x343)
  RELAY_MALFUNCTION_ADDRS = {0: (0x2E4, 0x191, 0x412)}
  FWD_BLACKLISTED_ADDRS = {2: [0x2E4, 0x412, 0x191]}

  LONGITUDINAL = False

  def test_diagnostics(self, stock_longitudinal: bool = True, ecu_disabled: bool = True):
    super().test_diagnostics(stock_longitudinal=stock_longitudinal, ecu_disabled=ecu_disabled)

  def test_block_aeb(self, stock_longitudinal: bool = True):
    super().test_block_aeb(stock_longitudinal=stock_longitudinal)

  def test_acc_cancel(self):
    """
      Regardless of controls allowed, never allow ACC_CONTROL if cancel bit isn't set
    """
    for controls_allowed in [True, False]:
      self.safety.set_controls_allowed(controls_allowed)
      for accel in np.arange(self.MIN_ACCEL - 1, self.MAX_ACCEL + 1, 0.1):
        self.assertFalse(self._tx(self._accel_msg_343(accel)))
        should_tx = np.isclose(accel, self.INACTIVE_ACCEL, atol=0.0001)
        self.assertEqual(should_tx, self._tx(self._accel_msg_343(accel, cancel_req=1)))


class TestToyotaStockLongitudinalTorque(TestToyotaStockLongitudinalBase, TestToyotaSafetyTorque):

  @property
  def SAFETY_PARAM(self):
    return self.EPS_SCALE | ToyotaSafetyFlags.STOCK_LONGITUDINAL


class TestToyotaStockLongitudinalAngle(TestToyotaStockLongitudinalBase, TestToyotaSafetyAngle):

  @property
  def SAFETY_PARAM(self):
    return self.EPS_SCALE | ToyotaSafetyFlags.STOCK_LONGITUDINAL | ToyotaSafetyFlags.LTA


class TestToyotaSecOcSafetyBase(TestToyotaSafetyBase):

  DBC = "toyota_secoc_pt_generated"

  TX_MSGS = TOYOTA_SECOC_TX_MSGS
  RELAY_MALFUNCTION_ADDRS = {0: (0x2E4, 0x191, 0x412, 0x131)}
  FWD_BLACKLISTED_ADDRS = {2: [0x2E4, 0x191, 0x412, 0x131]}

  @property
  def SAFETY_PARAM(self):
    return self.EPS_SCALE | ToyotaSafetyFlags.SECOC

  def test_diagnostics(self, ecu_disabled: bool = False):
    super().test_diagnostics(ecu_disabled=ecu_disabled)

  # This platform also has alternate brake and PCM messages, but same naming in the DBC, so same packers work

  def _user_gas_msg(self, gas):
    values = {"GAS_PEDAL_USER": gas}
    return self.packer.make_can_msg_safety("GAS_PEDAL", 0, values)

  # This platform sends both STEERING_LTA (same as other Toyota) and STEERING_LTA_2 (SecOC signed)
  # STEERING_LTA is checked for no-actuation by the base class, STEERING_LTA_2 is checked for no-actuation below

  def _lta_2_msg(self, req, req2, angle_cmd, torque_wind_down=100):
    values = {"STEER_REQUEST": req, "STEER_REQUEST_2": req2, "STEER_ANGLE_CMD": angle_cmd}
    return self.packer.make_can_msg_safety("STEERING_LTA_2", 0, values)

  def test_lta_2_steer_cmd(self):
    for engaged, req, req2, angle in itertools.product([True, False], [0, 1], [0, 1], np.linspace(-20, 20, 5)):
      self.safety.set_controls_allowed(engaged)

      should_tx = not req and not req2 and angle == 0
      self.assertEqual(should_tx, self._tx(self._lta_2_msg(req, req2, angle)), f"{req=} {req2=} {angle=}")

  def _accel_msg_183(self, accel):
    values = {"ACCEL_CMD": accel}
    return self.packer.make_can_msg_safety("ACC_CONTROL_2", 0, values)

  def _accel_msg(self, accel, cancel_req=0):
    return self._accel_msg_183(accel)


class TestToyotaSecOcSafetyStockLongitudinal(TestToyotaSecOcSafetyBase, TestToyotaStockLongitudinalBase):

  @property
  def SAFETY_PARAM(self):
    return self.EPS_SCALE | ToyotaSafetyFlags.STOCK_LONGITUDINAL | ToyotaSafetyFlags.SECOC


class TestToyotaSecOcSafety(TestToyotaSecOcSafetyBase):

  RELAY_MALFUNCTION_ADDRS = {0: (0x2E4, 0x191, 0x412, 0x131, 0x343, 0x183)}
  FWD_BLACKLISTED_ADDRS = {2: [0x2E4, 0x191, 0x412, 0x131, 0x343, 0x183]}

  @unittest.skip("test not applicable for cars without a DSU")
  def test_block_aeb(self, stock_longitudinal: bool = False):
    pass

  def test_343_actuation_blocked(self):
    """
    For SecOC cars, longitudinal acceleration must be sent in ACC_CONTROL_2, but all other ACC
    data remains in ACC_CONTROL. Verify no actuation is sent via ACC_CONTROL.
    """
    for controls_allowed in [True, False]:
      self.safety.set_controls_allowed(controls_allowed)
      for accel in np.arange(self.MIN_ACCEL - 1, self.MAX_ACCEL + 1, 0.1):
        should_tx = np.isclose(accel, self.INACTIVE_ACCEL, atol=0.0001)
        self.assertEqual(should_tx, self._tx(self._accel_msg_343(accel)))
        self.assertEqual(should_tx, self._tx(self._accel_msg_343(accel, cancel_req=1)))


if __name__ == "__main__":
  unittest.main()
