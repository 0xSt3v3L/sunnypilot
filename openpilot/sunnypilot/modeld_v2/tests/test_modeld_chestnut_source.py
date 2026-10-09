"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""

# Source-level guards for the chestnut changes in modeld_v2/modeld.py.
# Importing modeld.py needs tinygrad and the device runtime, so these tests inspect the AST instead.
# They pin the intent of each change; runtime behavior is verified on the device.

import ast
import os
import subprocess
import sys
import tempfile
import unittest
from pathlib import Path

MODELD_PATH = Path(__file__).resolve().parents[1] / 'modeld.py'
REPO_ROOT = Path(__file__).resolve().parents[4]
STDLIB_MODULES = {'os', 'sys', 'collections.abc', '__future__'}
# BASEDIR only imports os, so it may run before the env defaults take effect
EARLY_IMPORTS = STDLIB_MODULES | {'openpilot.common.basedir'}


def _parse() -> ast.Module:
  return ast.parse(MODELD_PATH.read_text())


def _main_body(tree: ast.Module) -> list[ast.stmt]:
  return next(n for n in tree.body if isinstance(n, ast.FunctionDef) and n.name == 'main').body


def _is_setdefault(node: ast.stmt, key: str, value: str) -> bool:
  return (isinstance(node, ast.Expr) and isinstance(node.value, ast.Call) and
          ast.unparse(node.value.func) == 'os.environ.setdefault' and
          [ast.literal_eval(a) for a in node.value.args] == [key, value])


def _import_modules(node: ast.stmt) -> list[str]:
  if isinstance(node, ast.Import):
    return [alias.name for alias in node.names]
  if isinstance(node, ast.ImportFrom):
    return [node.module or '']
  return []


def _is_third_party_import(node: ast.stmt, allowed: set[str] = STDLIB_MODULES) -> bool:
  return any(m not in allowed for m in _import_modules(node))


class TestModeldChestnutSource(unittest.TestCase):
  def _first_third_party_import(self, allowed: set[str] = STDLIB_MODULES) -> int:
    return next(i for i, n in enumerate(_parse().body) if _is_third_party_import(n, allowed))

  def _assert_env_default_before_imports(self, key: str, value: str):
    body = _parse().body
    idx = next((i for i, n in enumerate(body) if _is_setdefault(n, key, value)), None)
    self.assertIsNotNone(idx, f"{key} must default to '{value}' via os.environ.setdefault")
    self.assertLess(idx, self._first_third_party_import(), "tinygrad caches getenv, so the default must precede third-party imports")

  def test_power_limit_default_set_before_third_party_imports(self):
    self._assert_env_default_before_imports('AM_POWER_LIMIT', '100')

  def test_hcq2_defaults_to_legacy_runtime_before_third_party_imports(self):
    # HCQ2 is a tinygrad ContextVar read once at import time
    self._assert_env_default_before_imports('HCQ2', '0')

  def _legacy_path_if(self) -> tuple[int, ast.If]:
    for i, node in enumerate(_parse().body):
      if isinstance(node, ast.If) and ast.unparse(node.test) == "os.environ['HCQ2'] == '0'":
        return i, node
    self.fail("module-level `if os.environ['HCQ2'] == '0':` not found")

  def test_legacy_runtime_path_appended_from_basedir(self):
    _, node = self._legacy_path_if()
    self.assertEqual([ast.unparse(s) for s in node.body], ["sys.path.append(os.path.join(BASEDIR, 'tinygrad_repo'))"])

  def test_legacy_runtime_path_set_before_other_imports(self):
    idx, _ = self._legacy_path_if()
    self.assertLess(idx, self._first_third_party_import(EARLY_IMPORTS))

  def test_legacy_runtime_importable_with_device_layout(self):
    # On device, PYTHONPATH is the repo root and only tinygrad_repo/tinygrad is symlinked in (launch_chffrplus.sh),
    # so extra.hcq1 resolves only after modeld appends tinygrad_repo to sys.path.
    tinygrad_repo = REPO_ROOT / 'tinygrad_repo'
    code = ("import sys\n"
            "if sys.argv[1]: sys.path.append(sys.argv[1])\n"
            "import tinygrad.runtime.ops_amd as m\n"
            "print(m.AMDDevice.__module__)\n")
    with tempfile.TemporaryDirectory() as tmp:
      os.symlink(tinygrad_repo / 'tinygrad', Path(tmp) / 'tinygrad')
      env = {**os.environ, 'PYTHONPATH': tmp, 'HCQ2': '0'}

      def run(extra_path: str) -> subprocess.CompletedProcess:
        return subprocess.run([sys.executable, '-c', code, extra_path], env=env, cwd=tmp, capture_output=True, text=True, timeout=120)

      without = run('')
      self.assertNotEqual(without.returncode, 0)
      self.assertIn("No module named 'extra'", without.stderr)
      with_path = run(str(tinygrad_repo))
      self.assertEqual(with_path.returncode, 0, with_path.stderr)
      self.assertEqual(with_path.stdout.strip(), 'extra.hcq1.ops_amd_old')


if __name__ == '__main__':
  unittest.main()
