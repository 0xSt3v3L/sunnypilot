"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""
import ast
import pathlib
import unittest

# Parsed rather than imported: opendbc reads params out of a dict that this repo hands it, so the
# two sides can drift silently. A param opendbc asks for but this list omits simply reads as its
# default forever, which looks exactly like a feature that was never enabled.
REPO_ROOT = pathlib.Path(__file__).resolve().parents[5]
ALLOWLIST_SRC = REPO_ROOT / "openpilot/sunnypilot/selfdrive/car/interfaces.py"
OPENDBC_SRC = REPO_ROOT / "opendbc_repo/opendbc/sunnypilot/car/interfaces.py"


def _allowlisted_keys() -> set[str]:
  tree = ast.parse(ALLOWLIST_SRC.read_text())
  fn = next(n for n in ast.walk(tree)
            if isinstance(n, ast.FunctionDef) and n.name == "initialize_params")
  return {c.value for c in ast.walk(fn) if isinstance(c, ast.Constant) and isinstance(c.value, str)}


def _requested_keys() -> set[str]:
  """Every literal key opendbc pulls out of params_dict."""
  tree = ast.parse(OPENDBC_SRC.read_text())
  keys = set()
  for node in ast.walk(tree):
    if not isinstance(node, ast.Call) or not isinstance(node.func, ast.Attribute):
      continue
    if node.func.attr != "get" or not isinstance(node.func.value, ast.Name):
      continue
    if node.func.value.id != "params_dict" or not node.args:
      continue
    first = node.args[0]
    if isinstance(first, ast.Constant) and isinstance(first.value, str):
      keys.add(first.value)
  return keys


class TestParamsAllowlist(unittest.TestCase):
  def test_opendbc_source_is_present(self):
    self.assertTrue(OPENDBC_SRC.is_file(), f"{OPENDBC_SRC} missing; run git submodule update --init")

  def test_every_param_opendbc_reads_is_passed_through(self):
    requested = _requested_keys()
    self.assertTrue(requested, "found no params_dict.get() calls; the parser needs updating")
    missing = requested - _allowlisted_keys()
    self.assertEqual(set(), missing,
                     f"opendbc reads these params but initialize_params does not send them: {sorted(missing)}")


if __name__ == "__main__":
  unittest.main()
