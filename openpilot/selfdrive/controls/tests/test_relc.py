from openpilot.cereal import log, custom
from openpilot.cereal.services import SERVICE_LIST
from openpilot.common.realtime import DT_MDL
from openpilot.common.test import OpenpilotTestCase
from openpilot.selfdrive.controls.lib.relc import (RoadEdgeLaneChangeController, EDGE_REACTION_TIME, EDGE_CLEAR_TIME, MIN_SPEED,
                                                  VEHICLE_EDGE_MARGIN)

V_HIGH = MIN_SPEED + 2.
LEFT_EDGE_STDS, LEFT_EDGE_LANES = [0.0, 0.9], [0.0, 0.8, 0.8, 0.8]
NO_EDGE_STDS, NO_EDGE_LANES = [0.9, 0.9], [0.8, 0.8, 0.8, 0.8]


class Edge:
  def __init__(self, y):
    self.y = [y] * 33


CLOSE_EDGES = [Edge(-2.0), Edge(1.5)]
FAR_EDGES = [Edge(-10.0), Edge(10.0)]


def drive(c, stds, lanes, seconds, v_ego=V_HIGH, edges=CLOSE_EDGES):
  ret = (False, False)
  for _ in range(int(seconds / DT_MDL) + 1):
    ret = c.update(stds, lanes, v_ego, edges)
  return ret


class TestRELC(OpenpilotTestCase):
  def test_detects_left_edge_after_reaction_time(self):
    c = RoadEdgeLaneChangeController()
    assert drive(c, LEFT_EDGE_STDS, LEFT_EDGE_LANES, EDGE_REACTION_TIME + 0.1) == (True, False)

  def test_needs_reaction_time(self):
    c = RoadEdgeLaneChangeController()
    assert drive(c, LEFT_EDGE_STDS, LEFT_EDGE_LANES, EDGE_REACTION_TIME - 0.05) == (False, False)

  def test_clears_after_clear_time(self):
    c = RoadEdgeLaneChangeController()
    drive(c, LEFT_EDGE_STDS, LEFT_EDGE_LANES, EDGE_REACTION_TIME + 0.1)
    assert drive(c, NO_EDGE_STDS, NO_EDGE_LANES, EDGE_CLEAR_TIME + 0.1) == (False, False)

  def test_far_edges_are_ignored(self):
    c = RoadEdgeLaneChangeController()
    assert drive(c, LEFT_EDGE_STDS, LEFT_EDGE_LANES, EDGE_REACTION_TIME + 0.5, edges=FAR_EDGES) == (False, False)

  def test_inactive_below_min_speed(self):
    c = RoadEdgeLaneChangeController()
    assert drive(c, LEFT_EDGE_STDS, LEFT_EDGE_LANES, EDGE_REACTION_TIME + 0.5, v_ego=MIN_SPEED - 1.) == (False, False)

  def test_fill_road_edge(self):
    c = RoadEdgeLaneChangeController()
    drive(c, LEFT_EDGE_STDS, LEFT_EDGE_LANES, EDGE_REACTION_TIME + 0.1)
    road_edge = custom.ModelDataSP.RoadEdge.new_message()
    c.fill_road_edge(road_edge)
    assert road_edge.leftDetected and not road_edge.rightDetected
    assert abs(road_edge.leftProb - 1.0) < 1e-6
    assert abs(road_edge.rightProb - 0.1) < 1e-6
    assert abs(road_edge.leftClearance - (2.0 - VEHICLE_EDGE_MARGIN)) < 1e-6
    assert abs(road_edge.rightClearance - (1.5 - VEHICLE_EDGE_MARGIN)) < 1e-6

  def test_fill_road_edge_below_min_speed(self):
    c = RoadEdgeLaneChangeController()
    drive(c, LEFT_EDGE_STDS, LEFT_EDGE_LANES, 0.5, v_ego=MIN_SPEED - 1.)
    road_edge = custom.ModelDataSP.RoadEdge.new_message()
    c.fill_road_edge(road_edge)
    assert not road_edge.leftDetected and road_edge.leftProb == 0.0 and road_edge.leftClearance == 0.0

  def test_model_data_sp_service(self):
    msg = log.Event.new_message(modelDataSP={"roadEdge": {"leftDetected": True}})
    assert msg.modelDataSP.roadEdge.leftDetected
    assert SERVICE_LIST["modelDataSP"].frequency == 20.

  def test_modeld_publishes_road_edge(self):
    # modeld cannot be imported without its model runtime, so check its source
    import os
    from openpilot.common.basedir import BASEDIR
    with open(os.path.join(BASEDIR, "openpilot/selfdrive/modeld/modeld.py")) as f:
      modeld = f.read()
    assert '"modelDataSP"' in modeld
    assert "RELC.fill_road_edge(" in modeld
    assert "pm.send('modelDataSP'" in modeld
