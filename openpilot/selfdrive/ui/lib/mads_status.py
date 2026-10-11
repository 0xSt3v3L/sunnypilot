from openpilot.cereal import log, custom

OpenpilotState = log.SelfdriveState.OpenpilotState
MadsState = custom.SelfdriveStateSP.Mads.State


def mads_ui_status(ss, ss_sp) -> str:
  """UI status from selfdriveState and selfdriveStateSP: override, engaged, lat_only or disengaged."""
  mads = ss_sp.mads
  if ss.state in (OpenpilotState.preEnabled, OpenpilotState.overriding) or mads.state in (MadsState.paused, MadsState.overriding):
    return "override"
  if ss.enabled:
    return "engaged"
  if mads.available and mads.enabled:
    return "lat_only"
  return "disengaged"
