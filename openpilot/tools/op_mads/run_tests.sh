#!/usr/bin/env bash
# Run the op-chestnut-mads tests and lint in the macOS test copy (it has the uv env and macOS natives).
# Tests: every test_*.py under openpilot/ added or changed since the fork base, plus core upstream suites.
set -euo pipefail

WT=$(cd "$(dirname "$0")/../../.." && pwd)
# the copy for this upstream (msgq natives rebuilt after the 2ab952d5eb merge); op-mads-test-v0113 is the previous one
export OP_MADS_TEST=${OP_MADS_TEST:-$WT/../op-mads-test-up1019}
export OP_MADS_WT=$WT  # the sync script copies from this worktree into OP_MADS_TEST
TEST=$OP_MADS_TEST
SYNC=${OP_MADS_SYNC:-$WT/../op-mads-sync.sh}
BASE=f5715f8e81  # openpilot v0.11.3 nightly-chestnut build of 2ab952d5eb, the upstream the fork last merged
CORE=(
  openpilot.selfdrive.selfdrived.tests.test_state_machine
  openpilot.selfdrive.selfdrived.tests.test_alerts
  openpilot.selfdrive.ui.tests.test_soundd
  openpilot.selfdrive.monitoring.test_monitoring
  openpilot.selfdrive.controls.tests.test_desire_helper
  openpilot.common.tests.test_params
  openpilot.cereal.messaging.tests.test_services
)

"$SYNC"
cd "$TEST"

# the Python params bindings only know the keys compiled into libparams_c
if [ openpilot/common/params_keys.h -nt openpilot/common/libparams_c.dylib ]; then
  PATH=$PWD/.venv/bin:$PATH .venv/bin/scons -j8 openpilot/common/libparams_c.dylib | tail -1
fi

changed=$(cd "$WT" && { git diff --name-only --diff-filter=AM "$BASE" HEAD; git diff --name-only; git ls-files --others --exclude-standard; } | sort -u)
fork_tests=$(echo "$changed" | grep -E '^openpilot/.*/test_[a-z0-9_]+\.py$' | sed -e 's#/#.#g' -e 's#\.py$##' || true)
# shellcheck disable=SC2086
.venv/bin/python -m unittest $(printf '%s\n' $fork_tests "${CORE[@]}" | sort -u)

py_files=$(echo "$changed" | grep -E '^openpilot/.*\.py$' | while read -r f; do [ -f "$f" ] && echo "$f"; done || true)
# shellcheck disable=SC2086
.venv/bin/ruff check $py_files
