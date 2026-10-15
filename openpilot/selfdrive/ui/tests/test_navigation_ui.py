from pathlib import Path
from unittest.mock import patch

from openpilot.common.test import OpenpilotTestCase
from openpilot.selfdrive.ui.lib.navigation_page import page_lines

UI = Path(__file__).parents[1]


def source(path: str) -> str:
  # mici modules need a display to import, so their wiring is checked in the source
  return (UI / path).read_text()


class TestNavigationPage(OpenpilotTestCase):
  def test_url_on_wifi(self):
    with patch('openpilot.selfdrive.ui.lib.navigation_page.load_token', return_value='tok'), \
         patch('openpilot.selfdrive.ui.lib.navigation_page.web_url', return_value='http://172.20.10.2:8099/?t=tok') as url:
      self.assertEqual(page_lines(), ('http://172.20.10.2:8099/?t=tok', 'http://172.20.10.2:8099'))
    url.assert_called_once_with('tok')

  def test_no_wifi(self):
    with patch('openpilot.selfdrive.ui.lib.navigation_page.load_token', return_value='tok'), \
         patch('openpilot.selfdrive.ui.lib.navigation_page.web_url', return_value=None):
      url, text = page_lines()
    self.assertIsNone(url)
    self.assertIn('hotspot', text)


class TestNavigationWiring(OpenpilotTestCase):
  def test_road_view_shows_the_panel_with_a_destination(self):
    view = source('mici/onroad/augmented_road_view.py')
    self.assertIn('show_navigation = ui_state.navigation_enabled and ui_state.nav_destination_set and alert_to_render is None', view)
    self.assertIn('self._navigation.render(', view)
    self.assertIn('self._debug_overlay.render(self._content_rect)', view)  # the fork's overlay stays

  def test_ui_state(self):
    state = source('ui_state.py')
    self.assertIn('"navInstruction"', state)
    self.assertEqual(state.count('self.navigation_enabled'), 2)  # set at start and on every params update
    self.assertEqual(state.count('self.nav_destination_set'), 2)

  def test_developer_settings(self):
    dev = source('mici/layouts/settings/developer.py')
    self.assertIn('BigParamControl("navigation (alpha)", "NavigationEnabled"', dev)
    self.assertIn('("NavigationEnabled", self._navigation_toggle)', dev)
    self.assertIn('self._navigation_toggle)', dev.split('onroad_blocked_toggles = ')[1].split('\n')[0])
    self.assertIn('NavigationPage', dev)
    page = source('mici/layouts/settings/navigation_page.py')
    self.assertIn('QR(url)', page)
    self.assertIn('page_lines()', page)

  def test_torque_bar_keeps_mads(self):
    bar = source('mici/onroad/torque_bar.py')
    self.assertIn('TORQUE_ANGLE_SPAN * rect.width / 476', bar)
    self.assertIn('UIStatus.LAT_ONLY', bar)
