import tempfile
import unittest
from pathlib import Path

from openpilot.selfdrive.modeld.helpers import MODELS_DIR, modeld_pkl_path, nav_model_ready


class TestNavModel(unittest.TestCase):
  def test_paths(self):
    self.assertEqual(modeld_pkl_path(False), MODELS_DIR / 'driving_tinygrad.pkl')
    self.assertEqual(modeld_pkl_path(True), MODELS_DIR / 'big_driving_tinygrad.pkl')
    self.assertEqual(modeld_pkl_path(True, nav=True), MODELS_DIR / 'big_driving_nav_tinygrad.pkl')
    self.assertEqual(modeld_pkl_path(False, nav=True), MODELS_DIR / 'driving_tinygrad.pkl')  # the small model has no nav twin

  def test_ready_only_with_the_real_file(self):
    # without git-lfs on the device the checkout holds a 134-byte pointer; loading it would fail the big model
    with tempfile.TemporaryDirectory() as d:
      path = Path(d) / 'big_driving_nav_tinygrad.pkl'
      self.assertFalse(nav_model_ready(path))
      path.write_text('version https://git-lfs.github.com/spec/v1\noid sha256:52205044\nsize 822465506\n')
      self.assertFalse(nav_model_ready(path))
      with open(path, 'wb') as f:
        f.truncate(822465506)
      self.assertTrue(nav_model_ready(path))


if __name__ == '__main__':
  unittest.main()
