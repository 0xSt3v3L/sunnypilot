#!/usr/bin/env python3
"""Deploy op-chestnut-mads (or another fork branch) to the comma four and commit the binaries built there.

Run from the worktree of the branch:
  python3 openpilot/tools/op_mads/deploy.py --dry-run   # preflight only
  python3 openpilot/tools/op_mads/deploy.py [--branch op-nav-google] [--reboot] [--trailer "Co-Authored-By: ..."]

The merge and the build run in one ssh command: new Python can read params that only the rebuilt
libparams_c.so knows, so the device must never keep the new code with the old binaries. Another branch is
checked out on the device in that same command; each branch carries its own committed binaries.
"""
import argparse
import hashlib
import subprocess
import sys
from pathlib import Path

DEVICE = "comma_4"
DEVICE_DIR = "/data/openpilot"
BRANCH = "op-chestnut-mads"
REMOTE = "fork"
SCONS_LOG = "/data/tmp_mads/scons_deploy.log"
BUILD_TARGETS = ("openpilot/common/libparams_c.so", "openpilot/selfdrive/pandad/pandad", "openpilot/system/loggerd/loggerd")
# the root SConstruct rewrites these without rebuilding the firmware; they must keep matching the committed firmware
IGNORED_DIRTY = frozenset({"panda/board/obj/gitversion.h", "panda/board/obj/version"})
ROOT = Path(__file__).resolve().parents[3]


def parse_porcelain(text: str) -> list[tuple[str, str]]:
  return [(line[:2].strip(), line[3:]) for line in text.splitlines() if line.strip()]


def build_outputs(entries) -> list[str]:
  return [path for status, path in entries if status == "M" and path not in IGNORED_DIRTY]


def unexpected_dirty(entries) -> list[str]:
  return [path for _, path in entries if path not in IGNORED_DIRTY]


def parse_sha256sum(text: str) -> dict[str, str]:
  return {path: sha for sha, path in (line.split(maxsplit=1) for line in text.splitlines() if line.strip())}


def checksum_mismatches(local: dict[str, str], remote: dict[str, str]) -> list[str]:
  return [path for path, sha in remote.items() if local.get(path) != sha]


def device_onroad(is_onroad: str) -> bool:
  return is_onroad.strip() == "1"


def commit_message(device_head: str, paths: list[str], trailer: str) -> str:
  msg = ("build: libparams, pandad, loggerd and cereal generated code built on device\n\n" +
         f"Built on the comma 4 from {device_head} by openpilot/tools/op_mads/deploy.py:\n" +
         "".join(f"  {p}\n" for p in paths))
  return msg + (f"\n{trailer}\n" if trailer else "")


def update_command(branch: str, device_branch: str, commits: int) -> str:
  """Shell to bring the device repo to the fork's branch: a fast-forward on the same branch, else a checkout."""
  fetch = f"git fetch -q --depth {commits + 10} origin {branch}; "
  if branch == device_branch:
    return fetch + "git merge -q --ff-only FETCH_HEAD; "
  return fetch + f"git checkout -q -B {branch} FETCH_HEAD; "


def ancestry_required(branch: str, device_branch: str) -> bool:
  """The device HEAD must be an ancestor on the same branch. A switch checks the other branch out, and the device's
  own commits (the build outputs) are already pushed to the fork."""
  return branch == device_branch


def parse_device_state(out: str) -> tuple[bool, str, str, list[tuple[str, str]]]:
  """(onroad, branch, HEAD, status) from device_state's ssh output."""
  head_part, status = out.split("---\n", 1)
  lines = head_part.splitlines()
  return device_onroad(lines[0] if lines else ""), lines[-2].strip(), lines[-1].strip(), parse_porcelain(status)


def run(cmd: list[str], check: bool = True) -> str:
  return subprocess.run(cmd, cwd=ROOT, check=check, capture_output=True, text=True).stdout


def ssh(script: str, check: bool = True) -> str:
  return run(["ssh", "-o", "ConnectTimeout=8", "-o", "BatchMode=yes", DEVICE, script], check=check)


def device_state() -> tuple[bool, str, str, list[tuple[str, str]]]:
  return parse_device_state(ssh(f"cat /data/params/d/IsOnroad 2>/dev/null; echo; git -C {DEVICE_DIR} branch --show-current; " +
                                f"git -C {DEVICE_DIR} rev-parse HEAD; echo ---; git -C {DEVICE_DIR} status --porcelain"))


def fail(msg: str) -> int:
  print(f"ERROR: {msg}")
  return 1


def main(argv=None) -> int:
  parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
  parser.add_argument("--dry-run", action="store_true", help="preflight checks only")
  parser.add_argument("--reboot", action="store_true", help="reboot the device after a successful deploy")
  parser.add_argument("--trailer", default="", help="trailer line for the build commit")
  parser.add_argument("--branch", default=BRANCH, help=f"fork branch to deploy (default {BRANCH}); the device switches to it")
  args = parser.parse_args(argv)
  branch = args.branch

  # preflight
  if run(["git", "branch", "--show-current"]).strip() != branch:
    return fail(f"not on {branch}")
  if run(["git", "status", "--porcelain"]).strip():
    return fail("local worktree is not clean")
  head = run(["git", "rev-parse", "HEAD"]).strip()
  try:
    onroad, device_branch, device_head, device_status = device_state()
  except subprocess.CalledProcessError as e:
    return fail(f"cannot reach {DEVICE}: {e.stderr.strip()}")
  if onroad:
    return fail("device is onroad")
  if dirty := unexpected_dirty(device_status):
    return fail(f"device has unexpected changes: {dirty}")
  if ancestry_required(branch, device_branch) and \
     subprocess.run(["git", "merge-base", "--is-ancestor", device_head, head], cwd=ROOT).returncode != 0:
    return fail(f"device HEAD {device_head[:10]} is not an ancestor of local HEAD")
  if device_head == head and device_branch == branch:
    print("device is up to date")
    return 0
  # a switch checks the branch head out, so a shallow fetch of any depth does
  commits = int(run(["git", "rev-list", "--count", f"{device_head}..{head}"]).strip()) if device_branch == branch else 0
  print(f"deploy {commits} commits: {device_branch} {device_head[:10]} -> {branch} {head[:10]}")
  if args.dry_run:
    print("dry run: preflight passed")
    return 0

  # push, then update and build in one ssh command
  run(["git", "push", REMOTE, branch])
  merge_and_build = (f"set -e; cd {DEVICE_DIR}; " + update_command(branch, device_branch, commits) +
                     "export PATH=/usr/local/venv/bin:$PATH; mkdir -p /data/tmp_mads; " +
                     f"scons -j$(nproc) {' '.join(BUILD_TARGETS)} > {SCONS_LOG} 2>&1 || {{ tail -20 {SCONS_LOG}; exit 1; }}")
  build = subprocess.run(["ssh", DEVICE, merge_and_build], capture_output=True, text=True)
  if build.returncode != 0:
    print(build.stdout + build.stderr)
    return fail(f"merge or build failed; the device may hold the new code with old binaries, do not reboot it. Log: {SCONS_LOG}")
  print("merged and built on the device")

  # bring the build outputs back and verify them
  _, _, _, device_status = device_state()
  outputs = build_outputs(device_status)
  if untracked := [p for s, p in device_status if s == "??"]:
    print(f"ignoring untracked files on the device: {untracked}")
  if outputs:
    for path in outputs:
      run(["scp", "-q", f"{DEVICE}:{DEVICE_DIR}/{path}", str(ROOT / path)])
    remote = parse_sha256sum(ssh(f"cd {DEVICE_DIR} && sha256sum {' '.join(outputs)}"))
    local = {p: hashlib.sha256((ROOT / p).read_bytes()).hexdigest() for p in outputs}
    if bad := checksum_mismatches(local, remote):
      return fail(f"checksum mismatch after copy: {bad}")
    run(["git", "add", "-u", *outputs])
    run(["git", "commit", "-q", "-m", commit_message(head[:10], outputs, args.trailer)])
    run(["git", "push", REMOTE, branch])
    new_head = run(["git", "rev-parse", "HEAD"]).strip()
    ssh(f"cd {DEVICE_DIR} && git fetch -q --depth 10 origin {branch} && git reset -q --mixed FETCH_HEAD")
    print(f"committed {len(outputs)} build outputs as {new_head[:10]}")
  else:
    new_head = head
    print("no build outputs changed")

  onroad, device_branch, device_head, device_status = device_state()
  if device_branch != branch or device_head != new_head or unexpected_dirty(device_status):
    return fail(f"device {device_branch} {device_head[:10]} or status does not match local {branch} {new_head[:10]}")
  print(f"device HEAD matches {new_head[:10]}")

  if args.reboot:
    if onroad:
      return fail("device went onroad, not rebooting")
    ssh("sudo reboot", check=False)
    print("rebooting the device")
  else:
    print("reboot the device so every process loads the new binaries")
  return 0


if __name__ == "__main__":
  sys.exit(main())
