from openpilot.common.test import OpenpilotTestCase
from openpilot.tools.op_mads.deploy import (parse_porcelain, build_outputs, unexpected_dirty, parse_sha256sum, checksum_mismatches,
                                            device_onroad, commit_message, IGNORED_DIRTY, BUILD_TARGETS)

STATUS = " M openpilot/common/libparams_c.so\n?? openpilot/new_file\n M panda/board/obj/version\n M panda/board/obj/gitversion.h\n"


class TestDeployHelpers(OpenpilotTestCase):
  def test_parse_porcelain(self):
    assert parse_porcelain(STATUS) == [("M", "openpilot/common/libparams_c.so"), ("??", "openpilot/new_file"),
                                       ("M", "panda/board/obj/version"), ("M", "panda/board/obj/gitversion.h")]
    assert parse_porcelain("") == []

  def test_build_outputs_skip_panda_obj_and_untracked(self):
    assert build_outputs(parse_porcelain(STATUS)) == ["openpilot/common/libparams_c.so"]

  def test_unexpected_dirty(self):
    assert unexpected_dirty(parse_porcelain(STATUS)) == ["openpilot/common/libparams_c.so", "openpilot/new_file"]
    assert unexpected_dirty(parse_porcelain(" M panda/board/obj/version\n")) == []

  def test_ignored_dirty_is_only_panda_obj(self):
    assert all(p.startswith("panda/board/obj/") for p in IGNORED_DIRTY)

  def test_builds_params_and_cereal_consumers(self):
    assert "openpilot/common/libparams_c.so" in BUILD_TARGETS
    assert "openpilot/selfdrive/pandad/pandad" in BUILD_TARGETS
    assert "openpilot/system/loggerd/loggerd" in BUILD_TARGETS

  def test_checksums(self):
    remote = parse_sha256sum("aa  openpilot/a\nbb  openpilot/b\n")
    assert remote == {"openpilot/a": "aa", "openpilot/b": "bb"}
    assert checksum_mismatches({"openpilot/a": "aa", "openpilot/b": "bb"}, remote) == []
    assert checksum_mismatches({"openpilot/a": "aa", "openpilot/b": "xx"}, remote) == ["openpilot/b"]
    assert checksum_mismatches({"openpilot/a": "aa"}, remote) == ["openpilot/b"]

  def test_device_onroad(self):
    assert device_onroad("1")
    assert not device_onroad("0")
    assert not device_onroad("")

  def test_commit_message(self):
    msg = commit_message("abc1234", ["openpilot/common/libparams_c.so"], trailer="")
    assert msg.startswith("build: libparams, pandad, loggerd and cereal generated code built on device")
    assert "abc1234" in msg and "openpilot/common/libparams_c.so" in msg
    assert "Co-Authored-By" not in msg
    assert commit_message("abc1234", [], trailer="Co-Authored-By: x <y>").rstrip().endswith("Co-Authored-By: x <y>")
