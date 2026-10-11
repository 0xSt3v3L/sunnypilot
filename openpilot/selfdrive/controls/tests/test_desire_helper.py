from opendbc.car.structs import car
from openpilot.cereal import log
from openpilot.common.test import OpenpilotTestCase
from openpilot.selfdrive.controls.lib.desire_helper import DesireHelper

LaneChangeState = log.LaneChangeState


def start_left_lane_change(left_edge, right_edge=False):
  DH = DesireHelper()
  DH.update(car.CarState.new_message(vEgo=25., leftBlinker=True), True, 0.)
  assert DH.lane_change_state == LaneChangeState.preLaneChange
  nudge = car.CarState.new_message(vEgo=25., leftBlinker=True, steeringPressed=True, steeringTorque=1.)
  DH.update(nudge, True, 0., left_edge, right_edge)
  return DH.lane_change_state


class TestDesireHelperRoadEdge(OpenpilotTestCase):
  def test_left_edge_blocks_left_lane_change(self):
    assert start_left_lane_change(left_edge=True) == LaneChangeState.preLaneChange

  def test_no_edge_allows_lane_change(self):
    assert start_left_lane_change(left_edge=False) == LaneChangeState.laneChangeStarting

  def test_right_edge_does_not_block_left_lane_change(self):
    assert start_left_lane_change(left_edge=False, right_edge=True) == LaneChangeState.laneChangeStarting
