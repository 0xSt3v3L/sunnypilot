import ast
import unittest
from pathlib import Path

# modeld.py needs its model runtime to import, so these check its source
SOURCE = (Path(__file__).parents[1] / 'modeld.py').read_text()


def method(name: str) -> str:
  for node in ast.walk(ast.parse(SOURCE)):
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


if __name__ == '__main__':
  unittest.main()
