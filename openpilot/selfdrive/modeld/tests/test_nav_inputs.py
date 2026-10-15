import ast
import unittest
from pathlib import Path

# modeld.py needs its model runtime to import, so these check its source
SOURCE = (Path(__file__).parents[1] / 'modeld.py').read_text()


def method(name: str, cls: str | None = None) -> str:
  tree = ast.parse(SOURCE)
  if cls is not None:
    tree = next(n for n in ast.walk(tree) if isinstance(n, ast.ClassDef) and n.name == cls)
  for node in ast.walk(tree):
    if isinstance(node, ast.FunctionDef) and node.name == name:
      return ast.get_source_segment(SOURCE, node)
  raise AssertionError(f'{name} not found')


class TestNavInputs(unittest.TestCase):
  def test_inputs_keep_their_dtype(self):
    # the map input is uint8; packing every input as float32 would misread it
    pack = method('pack_inputs')
    self.assertIn('dtype.itemsize', pack)
    self.assertIn('_from_np_dtype(dtype)', pack)
    self.assertNotIn('dtype=np.float32, buffer', pack)

  def test_nav_inputs_are_copied_only_for_map_models(self):
    run = method('run')
    self.assertIn("for name in ('nav_map', 'navigation'):", run)
    self.assertIn('if name in self.npy:', run)
    self.assertIn('inputs.get(name, 0)', run)

  def test_navigation_enabled_picks_the_nav_model(self):
    main = method('main')
    self.assertIn('nav = params.get_bool("NavigationEnabled") and nav_model_ready(modeld_pkl_path(True, nav=True))', main)
    self.assertIn('ModelState(vipc_client_main.width, vipc_client_main.height, True, nav=nav)', main)
    init = method('__init__', cls='ModelState')
    self.assertIn('load_oob(modeld_pkl_path(chestnut, nav=nav), chestnut)', init)

  def test_nav_inputs_from_mapsd_and_navd(self):
    main = method('main')
    self.assertIn('VisionIpcClient("mapsd", VisionStreamType.VISION_STREAM_MAP, True)', main)
    self.assertIn('"navInstruction"', main)
    self.assertIn("inputs['nav_map'] = nav_map", main)
    self.assertIn("inputs['navigation'] = navigation_input(sm['navInstruction'].allManeuvers", main)
    self.assertIn('now - nav_map_time < NAV_MAP_TIMEOUT', main)


if __name__ == '__main__':
  unittest.main()
