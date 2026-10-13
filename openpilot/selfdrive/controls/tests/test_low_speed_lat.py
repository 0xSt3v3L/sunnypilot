import os
from openpilot.common.basedir import BASEDIR
from openpilot.common.test import OpenpilotTestCase
from openpilot.selfdrive.controls.lib.low_speed_lat import low_speed_lat_accel, DEADBAND, DEADBAND_RAMP, FULL_SPEED, MAX_SPEED


class TestLowSpeedLatAccel(OpenpilotTestCase):
  def test_stop_bias_is_zeroed(self):
    # routes 000000a3-a5: the model asks for +0.015 to +0.022 m/s^2 while stopping, which is 31-48 deg of wheel below 1 m/s
    for a in (0.015, 0.018, 0.022, -0.02):
      assert low_speed_lat_accel(a, 0.4) == 0.

  def test_stop_bias_with_a_blinker_is_zeroed(self):
    # routes 000000a8-a9: with either blinker on, +0.005 to +0.011 m/s^2 while stopping turned the wheel 22-38 deg right
    for a in (0.005, 0.009, 0.011):
      assert low_speed_lat_accel(a, 0.4) == 0.

  def test_real_low_speed_turns_are_unchanged(self):
    for a in (DEADBAND_RAMP, 0.15, 0.41, -1.27):
      assert low_speed_lat_accel(a, 1.0) == a

  def test_ramp_between_deadband_and_ramp_end(self):
    mid = (DEADBAND + DEADBAND_RAMP) / 2
    assert abs(low_speed_lat_accel(mid, 1.0) - mid * 0.5) < 1e-9
    assert low_speed_lat_accel(DEADBAND, 1.0) == 0.

  def test_fades_out_above_full_speed(self):
    halfway = (FULL_SPEED + MAX_SPEED) / 2
    out = low_speed_lat_accel(0.018, halfway)
    assert 0. < out < 0.018
    for v in (MAX_SPEED, 5., 30.):
      assert low_speed_lat_accel(0.018, v) == 0.018

  def test_never_grows_or_flips(self):
    for v in (0., 0.5, 1., 2., 2.5, 3., 10.):
      for i in range(-100, 101):
        a = i * 0.001
        out = low_speed_lat_accel(a, v)
        assert abs(out) <= abs(a) + 1e-12 and out * a >= 0.

  def test_modeld_uses_it_regardless_of_blinker(self):
    # modeld cannot be imported without its model runtime, so check its source
    with open(os.path.join(BASEDIR, "openpilot/selfdrive/modeld/modeld.py")) as f:
      modeld = f.read()
    assert "low_speed_lat_accel(model_output['action'][0,0], v_ego)" in modeld
    assert "blinker" not in modeld
