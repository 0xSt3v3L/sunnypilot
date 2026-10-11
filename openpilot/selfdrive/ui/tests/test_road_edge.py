import os

from openpilot.common.basedir import BASEDIR
from openpilot.common.test import OpenpilotTestCase
from openpilot.selfdrive.ui.lib.road_edge import road_edge_rgba


class TestRoadEdgeColor(OpenpilotTestCase):
  def test_red_while_engaged(self):
    assert road_edge_rgba(0.0, disengaged=False)[:3] == (255, 0, 0)

  def test_black_when_disengaged(self):
    assert road_edge_rgba(0.0, disengaged=True)[:3] == (0, 0, 0)

  def test_alpha_follows_confidence(self):
    assert road_edge_rgba(1.0, disengaged=False)[3] == 0
    assert road_edge_rgba(0.5, disengaged=False)[3] == int(0.5 * 255)
    # capped like the mici lane lines
    assert road_edge_rgba(0.0, disengaged=False)[3] == int(0.7 * 255)
    assert road_edge_rgba(2.0, disengaged=False)[3] == 0

  def test_mici_renderer_draws_road_edges_with_helper(self):
    # the mici renderer cannot be imported without a display, so check its source
    with open(os.path.join(BASEDIR, "openpilot/selfdrive/ui/mici/onroad/model_renderer.py")) as f:
      renderer = f.read()
    assert "rl.Color(*road_edge_rgba(float(self._road_edge_stds[i]), ui_state.status == UIStatus.DISENGAGED))" in renderer
