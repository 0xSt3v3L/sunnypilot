import os
from openpilot.common.basedir import BASEDIR
from openpilot.common.params import Params
from openpilot.common.test import OpenpilotTestCase
from openpilot.selfdrive.ui.lib.always_offroad import (always_offroad_button, set_always_offroad,
                                                       ENABLE_ONROAD, ENABLE_OFFROAD, DISABLE)


class TestAlwaysOffroadUI(OpenpilotTestCase):
  def test_button_onroad(self):
    assert always_offroad_button(started=True, always_offroad=False) == ENABLE_ONROAD

  def test_button_offroad(self):
    assert always_offroad_button(started=False, always_offroad=False) == ENABLE_OFFROAD

  def test_button_always_offroad(self):
    assert always_offroad_button(started=False, always_offroad=True) == DISABLE
    assert always_offroad_button(started=True, always_offroad=True) == DISABLE

  def test_set_and_clear(self):
    params = Params()
    assert set_always_offroad(params, True, engaged=False)
    assert params.get_bool("OffroadMode")
    assert set_always_offroad(params, False, engaged=False)
    assert not params.get_bool("OffroadMode")

  def test_refused_while_engaged(self):
    params = Params()
    assert not set_always_offroad(params, True, engaged=True)
    assert not params.get_bool("OffroadMode")

  def test_settings_uses_buttons(self):
    # the mici settings cannot be imported without a display, so check its source
    with open(os.path.join(BASEDIR, "openpilot/selfdrive/ui/mici/layouts/settings/settings.py")) as f:
      settings = f.read()
    for kind in ("ENABLE_ONROAD", "ENABLE_OFFROAD", "DISABLE"):
      assert f"always_offroad_button(ui_state.started, ui_state.always_offroad) == {kind}" in settings, kind
