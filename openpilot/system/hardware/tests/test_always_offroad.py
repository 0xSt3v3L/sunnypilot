import os
from openpilot.common.basedir import BASEDIR
from openpilot.common.params import Params, ParamKeyFlag
from openpilot.common.test import OpenpilotTestCase
from openpilot.system.hardware.hardwared import set_always_offroad_conditions


class TestAlwaysOffroad(OpenpilotTestCase):
  def test_param_cleared_on_manager_start(self):
    params = Params()
    params.put_bool("OffroadMode", True, block=True)
    assert params.get_bool("OffroadMode")
    params.clear_all(ParamKeyFlag.CLEAR_ON_MANAGER_START)
    assert not params.get_bool("OffroadMode")

  def test_blocks_startup_and_onroad(self):
    startup, onroad = {"device_booted": True}, {"ignition": True}
    set_always_offroad_conditions(startup, onroad, True)
    assert not all(startup.values()) and not all(onroad.values())

  def test_allows_onroad_when_cleared(self):
    startup, onroad = {"device_booted": True}, {"ignition": True}
    set_always_offroad_conditions(startup, onroad, True)
    set_always_offroad_conditions(startup, onroad, False)
    assert all(startup.values()) and all(onroad.values())

  def test_hardwared_reads_param(self):
    with open(os.path.join(BASEDIR, "openpilot/system/hardware/hardwared.py")) as f:
      hardwared = f.read()
    assert 'set_always_offroad_conditions(startup_conditions, onroad_conditions, params.get_bool("OffroadMode"))' in hardwared
