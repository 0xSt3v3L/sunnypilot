from openpilot.cereal import log, custom
from openpilot.common.test import OpenpilotTestCase
from openpilot.selfdrive.ui.lib.mads_status import mads_ui_status

OpenpilotState = log.SelfdriveState.OpenpilotState
MadsState = custom.SelfdriveStateSP.Mads.State


def ss(enabled=False, state=OpenpilotState.disabled):
  return log.SelfdriveState.new_message(enabled=enabled, state=state)


def ss_sp(enabled=False, state=MadsState.disabled, available=True):
  return custom.SelfdriveStateSP.new_message(mads={"enabled": enabled, "state": state, "available": available})


class TestMadsStatus(OpenpilotTestCase):
  def test_lateral_only(self):
    assert mads_ui_status(ss(), ss_sp(True, MadsState.enabled)) == "lat_only"

  def test_engaged(self):
    assert mads_ui_status(ss(True, OpenpilotState.enabled), ss_sp(True, MadsState.enabled)) == "engaged"

  def test_override(self):
    assert mads_ui_status(ss(), ss_sp(True, MadsState.paused)) == "override"
    assert mads_ui_status(ss(), ss_sp(True, MadsState.overriding)) == "override"
    assert mads_ui_status(ss(True, OpenpilotState.overriding), ss_sp(True, MadsState.enabled)) == "override"

  def test_disengaged(self):
    assert mads_ui_status(ss(), ss_sp()) == "disengaged"

  def test_status_without_mads(self):
    assert mads_ui_status(ss(True, OpenpilotState.enabled), ss_sp(available=False)) == "engaged"
    assert mads_ui_status(ss(), ss_sp(available=False)) == "disengaged"

  def test_lateral_only_lane_lines_match_confidence_ball_cyan(self):
    # the mici renderer cannot be imported without a display, so check its source
    import os
    from openpilot.common.basedir import BASEDIR
    with open(os.path.join(BASEDIR, "openpilot/selfdrive/ui/mici/onroad/model_renderer.py")) as f:
      renderer = f.read()
    assert "UIStatus.LAT_ONLY: rl.Color(0, 200, 200, 255)," in renderer
