from opendbc.car import DT_CTRL, structs
from opendbc.car.can_definitions import CanData
from opendbc.car.toyota.values import ToyotaFlags
from opendbc.sunnypilot.car.toyota.values import ToyotaFlagsSP


PRE_COLLISION_2_SIGNALS = (
  "DSS1GDRV", "DS1STAT2", "DS1STBK2", "PCSWAR", "PCSALM",
  "PCSOPR", "PCSABK", "PBATRGR", "PPTRGR", "IBTRGR",
  "CLEXTRGR", "IRLT_REQ", "BRKHLD", "AVSTRGR", "VGRSTRGR",
  "PREFILL", "PBRTRGR", "PCSDIS", "PBPREPMP",
)

ACTIVATION_FRAMES = round(0.5 / DT_CTRL)
SOURCE_TIMEOUT_NANOS = 100_000_000

GearShifter = structs.CarState.GearShifter


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


class AutoBrakeHoldController:
  def __init__(self, CP: structs.CarParams, CP_SP: structs.CarParamsSP):
    self.CP = CP
    self.CP_SP = CP_SP
    self.enabled = bool(CP_SP.flags & ToyotaFlagsSP.AUTO_BRAKE_HOLD)
    self.activation_frames = 0
    self.active = False

  def update_state(self, CS) -> None:
    disallowed_gear = CS.out.gearShifter in (GearShifter.park, GearShifter.reverse)
    release = (not CS.out.standstill or CS.out.gasPressed or CS.out.cruiseState.enabled or
               not CS.out.cruiseState.available or disallowed_gear)
    if release:
      self.activation_frames = 0
      self.active = False
    elif not self.active:
      if CS.out.brakePressed:
        self.activation_frames += 1
        self.active = self.activation_frames >= ACTIVATION_FRAMES
      else:
        self.activation_frames = 0

  def update(self, CS, packer, frame: int, now_nanos: int) -> list[CanData]:
    self.update_state(CS)
    source_age_nanos = now_nanos - CS.pre_collision_2_ts_nanos
    if (not self.enabled or not CS.pre_collision_2_seen or
        not 0 <= source_age_nanos <= SOURCE_TIMEOUT_NANOS or frame % 2 != 0):
      return []

    return [create_auto_brake_hold_command(packer, frame, CS.pre_collision_2, self.active)]
