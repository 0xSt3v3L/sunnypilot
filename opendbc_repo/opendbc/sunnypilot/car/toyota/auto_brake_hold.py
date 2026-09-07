from opendbc.car import structs
from opendbc.car.toyota.values import ToyotaFlags


def is_auto_brake_hold_available(CP: structs.CarParams) -> bool:
  incompatible = ToyotaFlags.RADAR_ACC | ToyotaFlags.SECOC
  return bool(
    CP.brand == "toyota" and
    CP.flags & ToyotaFlags.TSS2 and
    not CP.flags & incompatible and
    CP.openpilotLongitudinalControl
  )
