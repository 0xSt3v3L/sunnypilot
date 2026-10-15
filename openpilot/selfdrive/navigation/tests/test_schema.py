import unittest

import openpilot.cereal.messaging as messaging
from openpilot.cereal.services import SERVICE_LIST
from openpilot.common.params import Params


class TestNavigationSchema(unittest.TestCase):
  def test_services(self):
    for name in ('navInstruction', 'navRoute'):
      with self.subTest(name):
        self.assertIn(name, SERVICE_LIST)
        msg = messaging.new_message(name)
        self.assertEqual(msg.which(), name)

  def test_maneuvers(self):
    msg = messaging.new_message('navInstruction')
    msg.navInstruction.allManeuvers = [{'distance': 12.5, 'type': 'turn', 'modifier': 'left'}]
    self.assertEqual(msg.navInstruction.allManeuvers[0].modifier, 'left')

  def test_params(self):
    params = Params()
    self.assertFalse(params.get_bool('NavigationEnabled'))
    params.put('NavDestination', {'latitude': 25.0, 'longitude': 121.5})
    self.assertEqual(params.get('NavDestination'), {'latitude': 25.0, 'longitude': 121.5})
    params.remove('NavDestination')


if __name__ == '__main__':
  unittest.main()
