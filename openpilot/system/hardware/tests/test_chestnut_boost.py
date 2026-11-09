import os
import unittest

from openpilot.common.basedir import BASEDIR
from openpilot.system.hardware.chestnut.boost import CHECK_INTERVAL, RETRY_INTERVAL, BoostPin, parse_freq


class FakeRoot:
  """Stands in for `sudo sh -c`: records commands, models the PMIC frequency register."""
  def __init__(self, value: int | None = 0x07, fail_write: bool = False):
    self.value, self.fail_write, self.cmds = value, fail_write, []

  def __call__(self, cmd: str) -> str | None:
    self.cmds.append(cmd)
    if "0xa5" in cmd:  # a write: sec access unlock, then the frequency
      if not self.fail_write:
        self.value = int(cmd.rsplit("echo ", 1)[1].split(maxsplit=1)[0], 16)
      return ""
    return None if self.value is None else f"16a1: {self.value:02x}\n"

  @property
  def writes(self) -> list[str]:
    return [c for c in self.cmds if "0xa5" in c]


class TestParseFreq(unittest.TestCase):
  def test_parses_regmap_data(self):
    assert parse_freq("16a1: 07\n") == 7
    assert parse_freq("16a1: 00") == 0

  def test_rejects_garbage_and_out_of_range(self):
    assert parse_freq("") is None
    assert parse_freq("0010: XX") is None
    assert parse_freq("16a1: 08") is None
    assert parse_freq("16a2: 01") is None


class TestBoostPin(unittest.TestCase):
  def test_no_chestnut_never_touches_the_pmic(self):
    root = FakeRoot()
    pin = BoostPin(root)
    pin.update(False, True, 0.)
    pin.update(False, False, 100.)
    assert root.cmds == []

  def test_pins_600khz_once_and_verifies(self):
    root = FakeRoot(0x07)
    BoostPin(root).update(True, True, 0.)
    assert root.value == 0x00
    assert len(root.writes) == 1
    assert root.cmds[-1] == root.cmds[0]  # read back after the write

  def test_write_only_touches_sec_access_and_freq_registers(self):
    root = FakeRoot(0x01)
    BoostPin(root).update(True, True, 0.)
    write = root.writes[0]
    addresses = [part.split(" >")[0].split("echo ")[1] for part in write.split("&&") if "/address" in part]
    assert addresses == ["0x16d0", "0x16a1"]
    assert "echo 0xa5 >" in write and write.rstrip().endswith("echo 0x0 > /sys/kernel/debug/regmap/spmi0-02/data")

  def test_already_pinned_reads_without_writing(self):
    root = FakeRoot(0x00)
    BoostPin(root).update(True, True, 0.)
    assert len(root.cmds) == 1 and root.writes == []

  def test_checks_on_topology_change_or_after_interval(self):
    root = FakeRoot(0x00)
    pin = BoostPin(root)
    pin.update(True, True, 0.)
    pin.update(True, False, CHECK_INTERVAL - 1)
    assert len(root.cmds) == 1
    pin.update(True, True, CHECK_INTERVAL - 1)
    assert len(root.cmds) == 2
    pin.update(True, False, 2 * CHECK_INTERVAL)
    assert len(root.cmds) == 3

  def test_repins_after_the_driver_rewrites_it(self):
    root = FakeRoot(0x07)
    pin = BoostPin(root)
    pin.update(True, True, 0.)
    root.value = 0x07  # e.g. a USB device was added or removed
    pin.update(True, True, 1.)
    assert root.value == 0x00 and len(root.writes) == 2

  def test_read_failure_backs_off_without_writing(self):
    root = FakeRoot(None)
    pin = BoostPin(root)
    pin.update(True, True, 0.)
    pin.update(True, True, RETRY_INTERVAL - 1)
    assert root.writes == [] and len(root.cmds) == 1
    pin.update(True, False, RETRY_INTERVAL)
    assert len(root.cmds) == 2

  def test_failed_write_backs_off(self):
    root = FakeRoot(0x07, fail_write=True)
    pin = BoostPin(root)
    pin.update(True, True, 0.)
    assert len(root.writes) == 1
    pin.update(True, True, RETRY_INTERVAL - 1)
    assert len(root.writes) == 1
    pin.update(True, False, RETRY_INTERVAL)
    assert len(root.writes) == 2

  def test_reattach_checks_immediately(self):
    root = FakeRoot(0x00)
    pin = BoostPin(root)
    pin.update(True, True, 0.)
    pin.update(False, True, 1.)
    pin.update(True, False, 2.)
    assert len(root.cmds) == 2


class TestHardwared(unittest.TestCase):
  def test_hardwared_starts_the_thread(self):
    with open(os.path.join(BASEDIR, "openpilot/system/hardware/hardwared.py")) as f:
      hardwared = f.read()
    assert "threads.append(threading.Thread(target=chestnut_boost_thread, args=(end_event,)))" in hardwared


if __name__ == "__main__":
  unittest.main()
