import numpy as np

# Low-speed lateral deadband. modeld turns the model's lateral acceleration into curvature as a / max(1, v)^2, so near
# a stop a tiny request becomes a large curvature: on this car the model asks for +0.015 to +0.022 m/s^2 while stopping,
# and the torque controller (KP 250 at 1 m/s) turned the wheel 20-45 deg right before every stop. Below MAX_SPEED,
# requests smaller than DEADBAND are dropped and DEADBAND..DEADBAND_RAMP ramps back in; real turns ask for far more
# and are untouched, and nothing changes while a blinker is on.

DEADBAND = 0.03  # m/s^2
DEADBAND_RAMP = 0.06  # m/s^2, unchanged from here up
FULL_SPEED = 2.0  # m/s, full deadband below this
MAX_SPEED = 3.0  # m/s, no deadband from here up


def low_speed_lat_accel(lat_accel: float, v_ego: float, blinker: bool) -> float:
  if blinker or v_ego >= MAX_SPEED:
    return lat_accel
  scale = float(np.clip((MAX_SPEED - v_ego) / (MAX_SPEED - FULL_SPEED), 0., 1.))
  lo, hi = DEADBAND * scale, DEADBAND_RAMP * scale
  mag = abs(lat_accel)
  if mag >= hi:
    return lat_accel
  if mag <= lo:
    return 0.
  return lat_accel * (mag - lo) / (hi - lo)
