import re
import unittest
from pathlib import Path

import numpy as np

from opendbc.car import gen_empty_fingerprint, structs
from opendbc.car.car_helpers import interfaces
from opendbc.car.interfaces import get_torque_params
from opendbc.car.toyota.carcontroller import LOW_SPEED_STEER_DELTA_UP, low_speed_steer_delta_up
from opendbc.car.toyota.values import CAR

# op-chestnut-mads: the LKA torque ramps up faster at low speed, where intersection turns run out of time to reach full torque
TOYOTA_H = Path(__file__).parents[3] / "safety" / "modes" / "toyota.h"


def panda_rate_lookup() -> tuple[list[float], list[float]]:
  src = TOYOTA_H.read_text()
  block = src[src.index("TOYOTA_RATE_UP_LOOKUP"):]
  m = re.search(r"\{\s*\{([^{}]+)\},\s*\{([^{}]+)\},?\s*\}", block)
  return [float(v) for v in m.group(1).split(",")], [float(v) for v in m.group(2).split(",")]


def panda_rate(v_ego: float) -> int:
  # matches the safety: 1 m/s speed fudge, ROUND (half away from zero)
  xs, ys = panda_rate_lookup()
  return int(np.floor(np.interp(v_ego - 1., xs, ys) + 0.5))


def ramp(v_ego_raw: float, frames: int = 10) -> int:
  CarInterface = interfaces[CAR.TOYOTA_COROLLA_TSS2]
  CI = CarInterface(CarInterface.get_params(CAR.TOYOTA_COROLLA_TSS2, gen_empty_fingerprint(), [], False, False, docs=False))
  CI.CS.out.vEgoRaw = v_ego_raw
  CI.CS.out.vEgo = v_ego_raw
  CI.CS.out.steeringTorqueEps = 5000.  # far above any command: the 350 command-vs-EPS limit never binds
  CC = structs.CarControl()
  CC.latActive = True
  CC.actuators.torque = 1.
  CC = CC.as_reader()
  out = None
  for i in range(frames):
    out, _ = CI.apply(CC, i * 10_000_000)
  return out.torqueOutputCan


class TestLowSpeedSteerRate(unittest.TestCase):
  def test_rate_by_speed(self):
    assert LOW_SPEED_STEER_DELTA_UP == 24
    assert low_speed_steer_delta_up(0., 15) == 24
    assert low_speed_steer_delta_up(30 / 3.6, 15) == 24
    assert low_speed_steer_delta_up(35 / 3.6, 15) == 19
    assert low_speed_steer_delta_up(40 / 3.6, 15) == 15
    assert low_speed_steer_delta_up(30., 15) == 15

  def test_panda_curve(self):
    xs, ys = panda_rate_lookup()
    np.testing.assert_allclose(xs[:2], [30 / 3.6, 40 / 3.6], atol=1e-4)
    assert ys == [24., 15., 15.]

  def test_controller_never_exceeds_panda(self):
    for v in np.arange(0., 25., 0.01):
      assert low_speed_steer_delta_up(v, 15) <= panda_rate(v), v

  def test_controller_ramps(self):
    assert ramp(2.) == 240
    assert ramp(20.) == 150

  def test_jerk_within_iso(self):
    # same 0.5 s jerk as test_lateral_limits, with the low-speed rate
    max_lat_accel = get_torque_params()[CAR.TOYOTA_COROLLA_TSS2]["MAX_LAT_ACCEL_MEASURED"]
    accel_up = min(LOW_SPEED_STEER_DELTA_UP / 1500 * 0.5 / 0.01, 1.) * max_lat_accel
    assert accel_up / 0.5 <= 2.5 + 0.5


if __name__ == "__main__":
  unittest.main()
