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


def _load_function(name: str, **globs):
  # compile one module-level function from modeld.py with stubbed globals
  fn = next((n for n in _parse().body if isinstance(n, ast.FunctionDef) and n.name == name), None)
  if fn is None:
    raise AssertionError(f'{name}() not found at module level')
  ns = dict(globs)
  exec(compile(ast.Module(body=[fn], type_ignores=[]), str(MODELD_PATH), 'exec'), ns)
  return ns[name]


class _FakeDevice:
  def __init__(self):
    self.pending = {}


class _FakeRegistry:
  def __init__(self, devices, broken=False):
    self.devices, self.broken = devices, broken
    self._opened_devices = set(devices)

  def __getitem__(self, name):
    if self.broken:
      raise RuntimeError('registry broken')
    return self.devices[name]


class _GrowingRegistry(_FakeRegistry):
  # another thread opens a device while drop_chestnut iterates (the loader keeps running after a join timeout)
  def __getitem__(self, name):
    self.devices.setdefault('DISK', _FakeDevice())
    self._opened_devices.add('DISK')
    return super().__getitem__(name)


class _FakeLog:
  def __init__(self):
    self.exceptions = []

  def exception(self, msg):
    self.exceptions.append(msg)


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

  def test_hcq2_defaults_to_hcq2_runtime_before_third_party_imports(self):
    # HCQ2 is a tinygrad ContextVar read once at import time. The chestnut pkls embed hcq2 USB host programs,
    # so the legacy runtime (HCQ2=0) cannot run them; HCQ2=0 stays available as an external override.
    self._assert_env_default_before_imports('HCQ2', '1')

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

  def _load_big_try(self) -> ast.Try:
    fn = next(n for n in ast.walk(_parse()) if isinstance(n, ast.FunctionDef) and n.name == 'load_big')
    return next(n for n in fn.body if isinstance(n, ast.Try))

  def test_amd_wait_timeout_set_inside_guarded_load(self):
    # hcq2 ignores HCQDEV_WAIT_TIMEOUT_MS (commaai/openpilot#39012); inside the try, an import failure falls back to the small model
    body = [ast.unparse(s) for s in self._load_big_try().body]
    i_import = body.index('from tinygrad.runtime.ops_amd import AMDDevice')
    i_set = body.index('AMDDevice.wait_timeout_ms = 3000')
    i_load = next(i for i, s in enumerate(body) if 'ModelState(' in s)
    self.assertLess(i_import, i_set)
    self.assertLess(i_set, i_load)

  def test_chestnut_runtime_logged_before_load(self):
    body = [ast.unparse(s) for s in self._load_big_try().body]
    i_log = next((i for i, s in enumerate(body) if 'chestnut runtime: ' in s and 'AMDDevice.__module__' in s), None)
    self.assertIsNotNone(i_log, 'load_big must log which AMD runtime was imported')
    self.assertTrue(body[i_log].startswith('cloudlog.warning('))
    self.assertLess(i_log, next(i for i, s in enumerate(body) if 'ModelState(' in s))

  def test_legacy_wait_timeout_env_kept(self):
    # the legacy runtime (HCQ2=0) still reads HCQDEV_WAIT_TIMEOUT_MS (extra/hcq1/hcq.py)
    main = '\n'.join(ast.unparse(s) for s in _main_body(_parse()))
    self.assertIn("os.environ['HCQDEV_WAIT_TIMEOUT_MS'] = '3000'", main)

  def _fallback_handler(self) -> ast.ExceptHandler:
    for node in ast.walk(_parse()):
      if isinstance(node, ast.ExceptHandler) and 'big model failed, fall back to small' in ast.unparse(node):
        return node
    self.fail('big model fallback handler not found')

  def test_fallback_decides_on_model_chestnut(self):
    first = self._fallback_handler().body[0]
    self.assertIsInstance(first, ast.If)
    self.assertEqual(ast.unparse(first.test), 'not model.chestnut')
    self.assertIsInstance(first.body[0], ast.Raise)

  def test_fallback_does_not_read_chestnut_active_param(self):
    # ChestnutActive is written non-blocking in this same handler, so a read here can be stale
    self.assertNotIn("get_bool('ChestnutActive')", ast.unparse(self._fallback_handler()))

  def test_drop_chestnut_noop_without_amd(self):
    qcom = _FakeDevice()
    qcom.pending['x'] = 1
    _load_function('drop_chestnut', Device=_FakeRegistry({'QCOM': qcom}), cloudlog=_FakeLog())()
    self.assertEqual(qcom.pending, {'x': 1})

  def test_drop_chestnut_removes_amd_from_every_pending(self):
    amd, qcom, cpu = _FakeDevice(), _FakeDevice(), _FakeDevice()
    qcom.pending.update({amd: 7, cpu: 3})
    cpu.pending[amd] = 2
    _load_function('drop_chestnut', Device=_FakeRegistry({'AMD': amd, 'QCOM': qcom, 'CPU': cpu}), cloudlog=_FakeLog())()
    self.assertEqual(qcom.pending, {cpu: 3})
    self.assertEqual(cpu.pending, {})

  def test_drop_chestnut_tolerates_devices_opened_concurrently(self):
    amd, qcom = _FakeDevice(), _FakeDevice()
    qcom.pending[amd] = 7
    log = _FakeLog()
    _load_function('drop_chestnut', Device=_GrowingRegistry({'AMD': amd, 'QCOM': qcom}), cloudlog=log)()
    self.assertEqual(log.exceptions, [])
    self.assertEqual(qcom.pending, {})

  def test_drop_chestnut_never_raises(self):
    # it runs inside the fallback handler; raising there would kill modeld instead of falling back
    log = _FakeLog()
    _load_function('drop_chestnut', Device=_FakeRegistry({'AMD': _FakeDevice()}, broken=True), cloudlog=log)()
    self.assertEqual(log.exceptions, ['drop chestnut failed'])

  def test_fallback_drops_chestnut_before_clearing_active(self):
    body = [ast.unparse(s) for s in self._fallback_handler().body]
    self.assertIn('drop_chestnut()', body)
    self.assertLess(body.index('drop_chestnut()'), body.index("params.put_bool('ChestnutActive', False)"))

  def test_failed_big_load_drops_chestnut(self):
    chestnut_if = next(n for n in _main_body(_parse()) if isinstance(n, ast.If) and ast.unparse(n.test) == 'CHESTNUT'
                       and any('load_big' in ast.unparse(s) for s in n.body))
    body = [ast.unparse(s) for s in chestnut_if.body]
    i_put = body.index("params.put_bool('ChestnutActive', model is not None)")
    self.assertEqual(body[i_put + 1], 'if model is None:\n    drop_chestnut()')

  def _small_model_assign(self) -> tuple[list[ast.stmt], int]:
    body = _main_body(_parse())
    for i, node in enumerate(body):
      if isinstance(node, ast.Assign) and [ast.unparse(t) for t in node.targets] == ['small_model']:
        return body, i
    self.fail('small_model assignment not found in main()')

  def test_small_model_warmed_up_right_after_creation(self):
    body, i = self._small_model_assign()
    nxt = body[i + 1]
    self.assertIsInstance(nxt, ast.If)
    self.assertEqual(ast.unparse(nxt.test), 'small_model is not None')
    self.assertIn('small_model.warmup()', ast.unparse(nxt))

  def test_small_model_warmup_failure_only_fatal_without_big_model(self):
    body, i = self._small_model_assign()
    tries = [n for n in ast.walk(body[i + 1]) if isinstance(n, ast.Try)]
    self.assertEqual(len(tries), 1)
    try_body = '\n'.join(ast.unparse(s) for s in tries[0].body)
    self.assertIn('small_model.warmup()', try_body)
    self.assertIn('small model warmed up in', try_body)
    handler = tries[0].handlers[0]
    self.assertEqual(ast.unparse(handler.type), 'Exception')
    guard = handler.body[0]
    self.assertIsInstance(guard, ast.If)
    self.assertEqual(ast.unparse(guard.test), 'model is None')
    self.assertIsInstance(guard.body[0], ast.Raise)
    self.assertIn("cloudlog.exception('small model warmup failed')", ast.unparse(handler))


if __name__ == '__main__':
  unittest.main()
