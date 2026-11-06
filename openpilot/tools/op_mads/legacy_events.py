"""OnroadEvent names in logs from builds before the 89e7e0a4c4 merge.

Those builds logged the fork's events at @105-@124. The merge moved them to @150-@169 (upstream took @105, and
@106-@149 hold placeholders), so the current schema reads the old ordinals as the wrong names. All of those builds
ran AGNOS 19; the merge needs AGNOS 20.3, so initData.osVersion tells the two apart.
"""
from openpilot.cereal import log

LEGACY_FIRST, LEGACY_LAST, MOVED_BY = 105, 124, 45
_NAMES = {raw: name for name, raw in log.OnroadEvent.EventName.schema.enumerants.items()}


def is_legacy_log(msgs) -> bool:
  """True if the first initData in msgs comes from AGNOS 19 or older."""
  init = next((m.initData for m in msgs if m.which() == "initData"), None)
  if init is None:
    return False
  try:
    return int(init.osVersion.strip().split(".")[0]) < 20
  except ValueError:
    return False


def event_name(event, legacy: bool) -> str:
  raw = event.name.raw
  if legacy and LEGACY_FIRST <= raw <= LEGACY_LAST:
    return _NAMES[raw + MOVED_BY]
  return str(event.name)
