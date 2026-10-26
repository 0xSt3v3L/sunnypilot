"""EPS probe mode: closed-area tests of the EPS. First used to find that it faults above 1500; now a rate sweep: up and
down rates are allowed to EPS_PROBE_RATE_MAX per frame and eps_probe.py ramps its targets at the rate under test.

openpilot enables it at init in joystick debug mode. The CarParams flag (car controller limit) and the safety flag (panda
limit) are always set together, before the car controller is built.
"""
from opendbc.car import structs
from opendbc.car.interfaces import CarInterfaceBase
from opendbc.car.toyota.values import EPS_PROBE_RATE_MAX, EPS_PROBE_STEER_MAX, ToyotaFlags, ToyotaSafetyFlags

__all__ = ["EPS_PROBE_RATE_MAX", "EPS_PROBE_STEER_MAX", "apply_probe_rate", "eps_probe_active", "with_eps_probe"]

SafetyModel = structs.CarParams.SafetyModel
SteerControlType = structs.CarParams.SteerControlType


def eps_probe_active(CP: structs.CarParams) -> bool:
  return CP.brand == "toyota" and bool(CP.flags & ToyotaFlags.EPS_PROBE)


def _eligible(CP: structs.CarParams) -> bool:
  return (CP.brand == "toyota" and CP.steerControlType == SteerControlType.torque and not CP.dashcamOnly and
          not CP.flags & ToyotaFlags.SECOC and len(CP.safetyConfigs) > 0 and
          CP.safetyConfigs[-1].safetyModel == SafetyModel.toyota)


def apply_probe_rate(params, axes) -> None:
  """Probe mode: the up/down rate under test comes from testJoystick axis 2 (per frame), so the car controller ramps at
  exactly that rate; without one it keeps EPS_PROBE_RATE_MAX. The panda allows up to EPS_PROBE_RATE_MAX."""
  rate = int(round(axes[2])) if len(axes) > 2 else 0
  rate = min(rate, EPS_PROBE_RATE_MAX) if rate > 0 else EPS_PROBE_RATE_MAX
  params.STEER_DELTA_UP = rate
  params.STEER_DELTA_DOWN = rate


def with_eps_probe(CI: CarInterfaceBase) -> CarInterfaceBase:
  """Return a new interface with probe mode on, or CI unchanged if the car is not eligible."""
  CP = CI.CP
  if not _eligible(CP):
    return CI
  CP.flags |= int(ToyotaFlags.EPS_PROBE)
  CP.safetyConfigs[-1].safetyParam |= int(ToyotaSafetyFlags.EPS_PROBE)
  return type(CI)(CP)
