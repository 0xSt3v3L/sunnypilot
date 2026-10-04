"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""

# Source-level guards for the chestnut changes in modeld_v2/modeld.py.
# Importing modeld.py needs tinygrad and the device runtime, so these tests inspect the AST instead.
# They pin the intent of each change; runtime behavior is verified on the device.

import ast
import unittest
from pathlib import Path

MODELD_PATH = Path(__file__).resolve().parents[1] / 'modeld.py'
STDLIB_MODULES = {'os', 'collections.abc', '__future__'}


def _parse() -> ast.Module:
  return ast.parse(MODELD_PATH.read_text())


def _main_body(tree: ast.Module) -> list[ast.stmt]:
  return next(n for n in tree.body if isinstance(n, ast.FunctionDef) and n.name == 'main').body


def _is_setdefault(node: ast.stmt, key: str, value: str) -> bool:
  return (isinstance(node, ast.Expr) and isinstance(node.value, ast.Call) and
          ast.unparse(node.value.func) == 'os.environ.setdefault' and
          [ast.literal_eval(a) for a in node.value.args] == [key, value])


def _is_third_party_import(node: ast.stmt) -> bool:
  if isinstance(node, ast.Import):
    return any(alias.name not in STDLIB_MODULES for alias in node.names)
  if isinstance(node, ast.ImportFrom):
    return node.module not in STDLIB_MODULES
  return False


class TestModeldChestnutSource(unittest.TestCase):
  def test_power_limit_default_set_before_third_party_imports(self):
    body = _parse().body
    idx = next((i for i, n in enumerate(body) if _is_setdefault(n, 'AM_POWER_LIMIT', '100')), None)
    self.assertIsNotNone(idx, "AM_POWER_LIMIT must default to '100' via os.environ.setdefault")
    first_import = next(i for i, n in enumerate(body) if _is_third_party_import(n))
    self.assertLess(idx, first_import, "tinygrad caches getenv, so the default must precede third-party imports")

  def _fallback_handler(self) -> ast.ExceptHandler:
    for node in ast.walk(_parse()):
      if isinstance(node, ast.ExceptHandler) and 'chestnut failed, falling back to small' in ast.unparse(node):
        return node
    self.fail('chestnut fallback handler not found')

  def test_fallback_decides_on_model_chestnut(self):
    first = self._fallback_handler().body[0]
    self.assertIsInstance(first, ast.If)
    self.assertEqual(ast.unparse(first.test), 'not model.chestnut')
    self.assertIsInstance(first.body[0], ast.Raise)

  def test_fallback_does_not_read_chestnut_active_param(self):
    # ChestnutActive is written non-blocking in this same handler, so a read here can be stale
    self.assertNotIn("get_bool('ChestnutActive')", ast.unparse(self._fallback_handler()))


if __name__ == '__main__':
  unittest.main()
