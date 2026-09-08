"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""
import unittest

from openpilot.sunnypilot.modeld_v2.chestnut_recovery import (
  decide_recovery,
  is_controlling,
  RECOVERY_WAIT,
  RECOVERY_RESTART,
  RECOVERY_GIVE_UP,
  MAX_RECOVERY_ATTEMPTS,
)


class TestChestnutRecovery(unittest.TestCase):
  def test_wait_when_no_recovery_pending(self):
    # nothing to recover: never act, regardless of engagement or attempts
    self.assertEqual(decide_recovery(recovery_pending=False, engaged=False, attempts=0), RECOVERY_WAIT)
    self.assertEqual(decide_recovery(recovery_pending=False, engaged=True, attempts=5), RECOVERY_WAIT)

  def test_wait_when_engaged_even_if_pending(self):
    # safety: must never restart modeld while openpilot is actively controlling
    self.assertEqual(decide_recovery(recovery_pending=True, engaged=True, attempts=0), RECOVERY_WAIT)

  def test_restart_when_pending_and_disengaged_under_limit(self):
    self.assertEqual(decide_recovery(recovery_pending=True, engaged=False, attempts=0), RECOVERY_RESTART)
    self.assertEqual(decide_recovery(recovery_pending=True, engaged=False, attempts=MAX_RECOVERY_ATTEMPTS - 1), RECOVERY_RESTART)

  def test_give_up_when_attempts_reach_max(self):
    self.assertEqual(decide_recovery(recovery_pending=True, engaged=False, attempts=MAX_RECOVERY_ATTEMPTS), RECOVERY_GIVE_UP)
    self.assertEqual(decide_recovery(recovery_pending=True, engaged=False, attempts=MAX_RECOVERY_ATTEMPTS + 1), RECOVERY_GIVE_UP)

  def test_never_restart_while_engaged_regardless_of_attempts(self):
    # engaged blocks any action (restart or give-up) until disengaged
    for attempts in range(MAX_RECOVERY_ATTEMPTS + 2):
      self.assertEqual(decide_recovery(recovery_pending=True, engaged=True, attempts=attempts), RECOVERY_WAIT)


class TestIsControlling(unittest.TestCase):
  def test_not_controlling_when_disengaged_and_alive(self):
    self.assertFalse(is_controlling(carcontrol_alive=True, lat_active=False, long_active=False))

  def test_controlling_when_lat_active(self):
    self.assertTrue(is_controlling(carcontrol_alive=True, lat_active=True, long_active=False))

  def test_controlling_when_long_active(self):
    self.assertTrue(is_controlling(carcontrol_alive=True, lat_active=False, long_active=True))

  def test_controlling_when_carcontrol_not_alive(self):
    # unknown engagement (stale/absent carControl) must be treated as controlling -> never restart
    self.assertTrue(is_controlling(carcontrol_alive=False, lat_active=False, long_active=False))


if __name__ == "__main__":
  unittest.main()
