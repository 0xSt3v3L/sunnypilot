import unittest
from unittest.mock import MagicMock

from openpilot.system.manager.process_config import managed_processes


def params(enabled: bool):
  p = MagicMock()
  p.get_bool.side_effect = lambda key: enabled if key == 'NavigationEnabled' else False
  return p


class TestNavigationProcesses(unittest.TestCase):
  def test_navd_and_mapsd_run_onroad_with_navigation(self):
    for name in ('navd', 'mapsd'):
      with self.subTest(name):
        should_run = managed_processes[name].should_run
        self.assertTrue(should_run(True, params(True), None))
        self.assertFalse(should_run(True, params(False), None))
        self.assertFalse(should_run(False, params(True), None))

  def test_navweb_runs_with_navigation_also_offroad(self):
    # the destination is set from the phone before the drive
    should_run = managed_processes['navweb'].should_run
    self.assertTrue(should_run(False, params(True), None))
    self.assertTrue(should_run(True, params(True), None))
    self.assertFalse(should_run(False, params(False), None))


if __name__ == '__main__':
  unittest.main()
