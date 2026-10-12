# Always offroad, ported from sunnypilot: OffroadMode keeps the device offroad until it is cleared or the device reboots.
ENABLE_ONROAD = "enable_onroad"    # force offroad, shown while onroad
ENABLE_OFFROAD = "enable_offroad"  # force offroad, shown while offroad so ignition does not go onroad
DISABLE = "disable"                # exit always offroad, goes onroad again if the ignition is on


def always_offroad_button(started: bool, always_offroad: bool) -> str:
  """Which always offroad button the mici settings show; only one is visible at a time."""
  if always_offroad:
    return DISABLE
  return ENABLE_ONROAD if started else ENABLE_OFFROAD


def set_always_offroad(params, enable: bool, engaged: bool) -> bool:
  """Write OffroadMode unless openpilot or MADS is engaged. Returns whether it was written."""
  if engaged:
    return False
  params.put_bool("OffroadMode", enable, block=True)
  return True
