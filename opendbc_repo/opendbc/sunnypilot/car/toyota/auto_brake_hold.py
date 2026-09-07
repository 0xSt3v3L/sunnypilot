from opendbc.car import structs
from opendbc.car.toyota.values import ToyotaFlags


PRE_COLLISION_2_SIGNALS = (
  "DSS1GDRV", "DS1STAT2", "DS1STBK2", "PCSWAR", "PCSALM",
  "PCSOPR", "PCSABK", "PBATRGR", "PPTRGR", "IBTRGR",
  "CLEXTRGR", "IRLT_REQ", "BRKHLD", "AVSTRGR", "VGRSTRGR",
  "PREFILL", "PBRTRGR", "PCSDIS", "PBPREPMP",
)


def is_auto_brake_hold_available(CP: structs.CarParams) -> bool:
  incompatible = ToyotaFlags.RADAR_ACC | ToyotaFlags.SECOC
  return bool(
    CP.brand == "toyota" and
    CP.flags & ToyotaFlags.TSS2 and
    not CP.flags & incompatible and
    CP.openpilotLongitudinalControl
  )


def create_auto_brake_hold_command(packer, frame: int, stock_values: dict[str, float], active: bool):
  values = {signal: stock_values[signal] for signal in PRE_COLLISION_2_SIGNALS}
  if active:
    values = {
      "DSS1GDRV": 0x3FF,
      "PBRTRGR": frame % 730 < 727,
    }
  return packer.make_can_msg("PRE_COLLISION_2", 0, values)
