import re
import unittest
from pathlib import Path

import numpy as np

from opendbc.car import gen_empty_fingerprint, structs
from opendbc.car.car_helpers import interfaces
from opendbc.car.toyota.carcontroller import LOW_SPEED_STEER_DELTA, low_speed_steer_deltas
from opendbc.car.toyota.values import CAR

# op-chestnut-mads: the LKA torque ramps up and down faster at low speed, where intersection turns run out of time to
# reach full torque. 30 per frame is as fast as the EPS output follows (rate sweep, route 000000e6: 45 is no faster,
# 60 faults).
TOYOTA_H = Path(__file__).parents[3] / "safety" / "modes" / "toyota.h"


def panda_lookup(name: str) -> tuple[list[float], list[float]]:
  src = TOYOTA_H.read_text()
  block = src[src.index(name):]
  m = re.search(r"\{\s*\{([^{}]+)\},\s*\{([^{}]+)\},?\s*\}", block)
  return [float(v) for v in m.group(1).split(",")], [float(v) for v in m.group(2).split(",")]


def panda_rate(name: str, v_ego: float) -> int:
  # matches the safety: 1 m/s speed fudge, ROUND (half away from zero)
  xs, ys = panda_lookup(name)
  return int(np.floor(np.interp(v_ego - 1., xs, ys) + 0.5))


def make_ci(v_ego_raw: float):
  CarInterface = interfaces[CAR.TOYOTA_COROLLA_TSS2]
  CI = CarInterface(CarInterface.get_params(CAR.TOYOTA_COROLLA_TSS2, gen_empty_fingerprint(), [], False, False, docs=False))
  CI.CS.out.vEgoRaw = v_ego_raw
  CI.CS.out.vEgo = v_ego_raw
  CI.CS.out.steeringTorqueEps = 5000.  # far above any command: the 350 command-vs-EPS limit never binds
  return CI


def drive(CI, torque: float, frames: int, start_frame: int = 0) -> int:
  CC = structs.CarControl()
  CC.latActive = True
  CC.actuators.torque = torque
  CC = CC.as_reader()
  out = None
  for i in range(frames):
    out, _ = CI.apply(CC, (start_frame + i) * 10_000_000)
  return out.torqueOutputCan


class TestLowSpeedSteerRate(unittest.TestCase):
  def test_rates_by_speed(self):
    assert LOW_SPEED_STEER_DELTA == 30
    cases = [(0., 30, 30), (20., 30, 30), (25., 27, 27), (30., 24, 25), (35., 19, 25), (40., 15, 25), (100., 15, 25)]
    for kph, up, down in cases:
      assert low_speed_steer_deltas(kph / 3.6, 15, 25) == (up, down), kph

  def test_panda_curves(self):
    xs, ys = panda_lookup("TOYOTA_RATE_UP_LOOKUP")
    np.testing.assert_allclose(xs, [20 / 3.6, 30 / 3.6, 40 / 3.6], atol=1e-4)
    assert ys == [30., 24., 15.]

  def test_controller_never_exceeds_panda(self):
    for v in np.arange(0., 25., 0.01):
      up, down = low_speed_steer_deltas(v, 15, 25)
      assert up <= panda_rate("TOYOTA_RATE_UP_LOOKUP", v), v
      # TorqueMotorLimited: max_rate_down (25) is the least a command above EPS + 350 must drop, so the release must not
      # be slower than it
      assert down >= 25, v

  def test_controller_ramps(self):
    assert drive(make_ci(2.), 1., 10) == 300
    assert drive(make_ci(20.), 1., 10) == 150

  def test_controller_releases(self):
    CI = make_ci(2.)
    assert drive(CI, 1., 60) == 1500
    assert drive(CI, 0., 1, start_frame=60) == 1470
    CI = make_ci(20.)
    assert drive(CI, 1., 120) == 1500
    assert drive(CI, 0., 1, start_frame=120) == 1475


if __name__ == "__main__":
  unittest.main()
