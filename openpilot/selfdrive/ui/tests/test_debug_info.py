from openpilot.cereal import log, custom
from opendbc.car.structs import car
from openpilot.common.test import OpenpilotTestCase
from openpilot.selfdrive.ui.lib.debug_info import debug_lines

MadsState = custom.SelfdriveStateSP.Mads.State


def lines(ss=None, ss_sp=None, cs=None, rs=None, mv=None, md=None):
  return debug_lines(ss or log.SelfdriveState.new_message(),
                     ss_sp or custom.SelfdriveStateSP.new_message(mads={"available": True}),
                     cs or car.CarState.new_message(),
                     rs or log.RadarState.new_message(),
                     mv or log.ModelDataV2.new_message(),
                     md or custom.ModelDataSP.new_message())


class TestDebugInfo(OpenpilotTestCase):
  def test_defaults(self):
    assert lines() == ["mads disabled  lat off", "steer +0.0  trq +0", "lead --", "plan --", "edge L0 0.00  R0 0.00", "turn --"]

  def test_mads_lateral_only(self):
    ss_sp = custom.SelfdriveStateSP.new_message(mads={"available": True, "state": MadsState.enabled, "active": True})
    assert lines(ss_sp=ss_sp)[0] == "mads enabled  lat on"

  def test_without_mads(self):
    ss_sp = custom.SelfdriveStateSP.new_message(mads={"available": False})
    assert lines(ss=log.SelfdriveState.new_message(active=True), ss_sp=ss_sp)[0] == "mads n/a  lat on"

  def test_steering(self):
    cs = car.CarState.new_message(steeringAngleDeg=-12.34, steeringTorque=85.)
    assert lines(cs=cs)[1] == "steer -12.3  trq +85"

  def test_lead(self):
    rs = log.RadarState.new_message(leadOne={"present": True, "dRel": 15.24, "vRel": 0.42})
    assert lines(rs=rs)[2] == "lead 15.2m +0.4"

  def test_plan_end(self):
    mv = log.ModelDataV2.new_message(position={"x": [0., 10., 45.4]})
    assert lines(mv=mv)[3] == "plan 45m"

  def test_road_edge(self):
    md = custom.ModelDataSP.new_message(roadEdge={"leftDetected": True, "leftProb": 0.625, "rightProb": 0.1})
    assert lines(md=md)[4] == "edge L1 0.62  R0 0.10"

  def test_lane_turn(self):
    md = custom.ModelDataSP.new_message(laneTurnDirection=custom.ModelDataSP.TurnDirection.turnLeft)
    assert lines(md=md)[5] == "turn left"

  def test_overlay_drawn_only_in_debug_mode(self):
    # the mici renderer cannot be imported without a display, so check its source
    import os
    from openpilot.common.basedir import BASEDIR
    with open(os.path.join(BASEDIR, "openpilot/selfdrive/ui/mici/onroad/augmented_road_view.py")) as f:
      view = f.read()
    assert "if ui_state.show_debug_info:" in view
    assert "self._debug_overlay.render(self._content_rect)" in view
