import numpy as np

# Low-speed lateral deadband. modeld turns the model's lateral acceleration into curvature as a / max(1, v)^2, so near
# a stop a tiny request becomes a large curvature: on this car the model asks for +0.015 to +0.022 m/s^2 while stopping,
# and the torque controller (KP 250 at 1 m/s) turned the wheel 20-45 deg right before every stop. Below MAX_SPEED,
# requests smaller than DEADBAND are dropped and DEADBAND..DEADBAND_RAMP ramps back in; real turns ask for far more
# and are untouched. This also applies with a blinker on: stopping with either blinker, the model asked for +0.005 to
# +0.011 m/s^2 and the wheel turned 22-38 deg right (routes 000000a8-a9).

DEADBAND = 0.03  # m/s^2
DEADBAND_RAMP = 0.06  # m/s^2, unchanged from here up
FULL_SPEED = 2.0  # m/s, full deadband below this
MAX_SPEED = 3.0  # m/s, no deadband from here up


def low_speed_lat_accel(lat_accel: float, v_ego: float) -> float:
  if v_ego >= MAX_SPEED:
    return lat_accel
  scale = float(np.clip((MAX_SPEED - v_ego) / (MAX_SPEED - FULL_SPEED), 0., 1.))
  lo, hi = DEADBAND * scale, DEADBAND_RAMP * scale
  mag = abs(lat_accel)
  if mag >= hi:
    return lat_accel
  if mag <= lo:
    return 0.
  return lat_accel * (mag - lo) / (hi - lo)


# Low-speed swing shaping. At 7-22 km/h the wheel swings left and right with a ~1.3 s period because the model's request
# itself swings by a few hundredths of a m/s^2, which a / v^2 turns into 5-10 deg of wheel; the torque controller only
# follows it (routes 000000a2-a9: request and wheel swing equally, the wheel 0.4 s behind). Split the request into a
# slow part (low-passed) that passes, so steady gentle bends keep all of it, and a fast part that is scaled by
# SHAPE_FAST_GAIN. Only small requests are shaped, so real turns are untouched. Offline on routes 000000a2-a9 this keeps
# 55% of the swing band with no delay on 94 low-speed turns.

SHAPE_TAU = 0.5  # s, low-pass time constant for the slow part
SHAPE_FAST_GAIN = 0.25
SHAPE_SMALL = 0.1  # m/s^2, full shaping below, none from 1.5x up
SHAPE_SPEEDS = [1.0, 2.0, 6.0, 7.0]  # m/s, shaping ramps in, holds, ramps out


class LowSpeedLatShaper:
  def __init__(self, dt: float):
    self.alpha = dt / (SHAPE_TAU + dt)
    self.slow: float | None = None

  def update(self, lat_accel: float, v_ego: float) -> float:
    self.slow = lat_accel if self.slow is None else self.slow + self.alpha * (lat_accel - self.slow)
    weight = float(np.interp(v_ego, SHAPE_SPEEDS, [0., 1., 1., 0.]) *
                   np.interp(abs(lat_accel), [SHAPE_SMALL, 1.5 * SHAPE_SMALL], [1., 0.]))
    gain = 1. - (1. - SHAPE_FAST_GAIN) * weight
    return self.slow + gain * (lat_accel - self.slow)
