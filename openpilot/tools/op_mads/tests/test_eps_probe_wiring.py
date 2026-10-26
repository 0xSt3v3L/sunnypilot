import re
from pathlib import Path

from openpilot.common.test import OpenpilotTestCase

# card, controlsd and joystickd need hardware and messaging to import: check their wiring by reading the source
ROOT = Path(__file__).resolve().parents[3]


def source(rel: str) -> str:
  return (ROOT / rel).read_text()


class TestEpsProbeWiring(OpenpilotTestCase):
  def test_card_applies_probe(self):
    src = source("selfdrive/car/card.py")
    m = re.search(r'if self\.params\.get_bool\("JoystickDebugMode"\):\n\s+self\.CI = with_eps_probe\(self\.CI\)', src)
    assert m is not None
    assert m.start() > src.index("self.CI = get_car(")
    assert m.start() < src.index("self.RI = interfaces[")

  def test_controlsd_refuses_probe_session(self):
    src = source("selfdrive/controls/controlsd.py")
    lat_active = src[src.index("CC.latActive = "):src.index("CC.longActive = ")]
    assert "not eps_probe_active(self.CP)" in lat_active

  def test_joystickd_follows_mads(self):
    src = source("tools/joystick/joystickd.py")
    assert "'selfdriveStateSP'" in src
    lat_active = src[src.index("CC.latActive = "):src.index("CC.longActive = ")]
    assert "mads_lat_active(sm['selfdriveState'], sm['selfdriveStateSP'])" in lat_active
    assert "sm['selfdriveState'].active" not in lat_active

  def test_no_device_joystick_publisher(self):
    # eps_probe.py is the only testJoystick publisher on the device
    assert "joystick_control" not in source("system/manager/process_config.py")

  def test_card_applies_probe_rate(self):
    src = source("selfdrive/car/card.py")
    assert "'testJoystick'" in src
    m = re.search(r"if eps_probe_active\(self\.CP\):\n\s+apply_probe_rate\(self\.CI\.CC\.params, self\.sm\['testJoystick'\]\.axes\)", src)
    assert m is not None
    assert m.start() < src.index("self.CI.apply(CC, now_nanos)")
