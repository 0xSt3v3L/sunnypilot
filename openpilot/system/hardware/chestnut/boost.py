"""Pin the PMI8998 OTG boost, which makes VBUS for the chestnut's USB bridge, at 600 kHz while a chestnut is attached.

dwc3-msm sets the boost switching frequency from the bMaxPower of each USB device added on either controller: 2 MHz up
to 100 mA, 800 kHz above. The chestnut reports 0 mA, so a hot attach leaves the boost at 2 MHz; after boot the modem
(200 mA, enumerated later) leaves it at 800 kHz. The switching noise causes USB3 link errors (0/min at 600 kHz, ~3/min
at 800 kHz, ~200/min at 2 MHz), and about one in 600 link errors aborts a big model transfer.
"""
import re
import subprocess
import time

from openpilot.common.hardware.usb import get_usb_state, get_usb_topology, is_chestnut_usb_id
from openpilot.common.swaglog import cloudlog

REGMAP = "/sys/kernel/debug/regmap/spmi0-02"  # PMI8998
FREQ_REG = 0x16A1  # CFG_BUCKBOOST_FREQ_SELECT_BOOST: 600 + 200 * value kHz
SEC_ACCESS_REG = 0x16D0  # unlocks the next write to a register at offset >= 0xA0
TARGET = 0x00  # 600 kHz
CHECK_INTERVAL = 30.  # the driver rewrites it on USB device add/remove, which also changes the topology
RETRY_INTERVAL = 60.

READ_CMD = f"echo {FREQ_REG:#x} > {REGMAP}/address && echo 1 > {REGMAP}/count && cat {REGMAP}/data"


def write_cmd(value: int) -> str:
  return (f"echo {SEC_ACCESS_REG:#x} > {REGMAP}/address && echo 0xa5 > {REGMAP}/data && " +
          f"echo {FREQ_REG:#x} > {REGMAP}/address && echo {value:#x} > {REGMAP}/data")


def parse_freq(out: str) -> int | None:
  m = re.fullmatch(rf"\s*{FREQ_REG:x}:\s*([0-9a-f]{{2}})\s*", out, re.IGNORECASE)
  value = int(m.group(1), 16) if m else None
  return value if value is not None and value <= 7 else None


def khz(value: int | None) -> int | None:
  return None if value is None else 600 + 200 * value


def run_root(cmd: str) -> str | None:
  try:
    ret = subprocess.run(["sudo", "sh", "-c", cmd], capture_output=True, text=True, timeout=5)
  except (OSError, subprocess.TimeoutExpired):
    return None
  return ret.stdout if ret.returncode == 0 else None


class BoostPin:
  def __init__(self, run=run_root):
    self.run = run
    self.next_check = 0.
    self.backoff_until = 0.  # after a failure, also holds over topology changes (a flapping link)

  def read(self) -> int | None:
    return parse_freq(self.run(READ_CMD) or "")

  def update(self, chestnut_present: bool, topology_changed: bool, now: float) -> None:
    if not chestnut_present:
      self.next_check = 0.
      return
    if now < self.backoff_until or (not topology_changed and now < self.next_check):
      return
    self.next_check = now + CHECK_INTERVAL

    value = self.read()
    if value is None:
      cloudlog.error("chestnut boost: frequency register read failed")
      self.backoff_until = now + RETRY_INTERVAL
      return
    if value == TARGET:
      return

    self.run(write_cmd(TARGET))
    after = self.read()
    cloudlog.event("chestnut boost pinned", prev_khz=khz(value), khz=khz(after), ok=after == TARGET)
    if after != TARGET:
      self.backoff_until = now + RETRY_INTERVAL


def chestnut_boost_thread(end_event):
  pin, topology, present = BoostPin(), None, False
  while not end_event.is_set():
    try:
      new_topology = get_usb_topology()
      changed = new_topology != topology
      if changed:
        topology = new_topology
        present = any(is_chestnut_usb_id(d["vendorId"], d["productId"]) for d in get_usb_state())
      pin.update(present, changed, time.monotonic())
    except Exception:
      cloudlog.exception("chestnut boost thread")
    end_event.wait(1.)
