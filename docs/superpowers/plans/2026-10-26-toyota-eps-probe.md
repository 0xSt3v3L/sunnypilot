# Toyota EPS Probe Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** A test branch that lets the user command 1500-3000 LKA torque on the Corolla only in joystick probe mode, with a probe script that runs the step profile, aborts on trouble and logs per-level verdicts.

**Architecture:** One opendbc helper (`opendbc/car/toyota/eps_probe.py`) owns eligibility and sets a CarParams flag plus a panda safety flag together; card applies it at init when `JoystickDebugMode` is on and rebuilds the interface. Panda safety raises `max_torque` to 3000 only with the flag (debug builds only). `joystickd` follows MADS lateral, `controlsd` refuses lateral in a probe session, and `eps_probe.py` publishes `testJoystick`.

**Tech Stack:** Python (openpilot, opendbc), panda safety C (MISRA), unittest, scons + arm-none-eabi 13.2 for the firmware.

**Spec:** `docs/superpowers/specs/2026-10-26-toyota-eps-above-1500-probe-design.md`

## Global Constraints

- Branch `op-chestnut-mads-eps-probe` from `c4eedce50a`, implemented in the session worktree
  `/Users/steve/openpilot_repos/openpilot/.claude/worktrees/hkg-low-speed-torque-eaa57a` (writes outside it are blocked).
  `opendbc_repo/` is vendored in this repo (no submodule): one commit covers opendbc and openpilot.
- Probe limit 3000 (`EPS_PROBE_STEER_MAX`), normal 1500. Rates 15/25, `STEER_ERROR_MAX` / `max_torque_error` 350,
  `max_rt_delta` 450, `MAX_USER_TORQUE` 500 and the 100 deg/s fault avoidance unchanged.
- `ToyotaFlags.EPS_PROBE = 4096`, `ToyotaSafetyFlags.EPS_PROBE = 16 << 8`; the safety flag is read only under `ALLOW_DEBUG`.
- Probe mode = `JoystickDebugMode` (existing param, no new params, so no `libparams_c.so` rebuild).
- Profile: levels 1500, 1800, 2100, 2400, 2700, 3000; each left then right; hold 1.5 s after the ramp; rest 2.5 s at 0.
- Aborts: steer fault (temp or perm), `steeringPressed`, `|steeringRateDeg| > 100`, `|steeringAngleDeg| > 450`,
  `vEgo` > 0.3 m/s (stage 1) or > 5 km/h (stage 2), `latActive` false > 0.5 s after it was up, no `latActive` within 3 s
  of start ("no lateral at this speed"), Ctrl-C.
- Verdicts: `ACCEPT` (hold median |cmd| >= target - 100 and median eps/cmd >= 1.2), `NO-RESPONSE` (max |eps| < 100 over
  the hold), `FAULT` (any steer fault in the step), else `CLIP`.
- JSONL log to `/data/eps_probe/<route>--<time>.jsonl`.
- No push, no deploy, no reboot without the user's approval. Local commits only.
- Test commands: opendbc tests `cd opendbc_repo && PYTHONPATH=$PWD /Users/steve/openpilot_repos/openpilot/.worktrees/opendbc-env/.venv/bin/python -m unittest <module>`;
  openpilot tests `openpilot/tools/op_mads/run_tests.sh` (syncs into the shared test copy `.worktrees/op-mads-test-v0113`).

## Review Focus

1. The probe script is started on a drive without probe mode (flag absent): it must refuse, since axis 0.5 would then mean 750, not 1500. Test in Task 4.
2. `JoystickDebugMode` is cleared while onroad: `controlsd` takes over and must not steer with the raised limit. Test in Task 1 (`eps_probe_active`) and Task 3 (wiring).
3. Another brand happens to have flag bit 4096 set: `eps_probe_active` must stay false. Test in Task 1.
4. A release (non-debug) panda build gets the flag: it must keep 1500. Covered by the `ALLOW_DEBUG` guard plus the release build compiling (Task 2).
5. The SSH session dies mid-run: `joystickd` zeroes the axes after 0.2 s without `testJoystick` (existing behavior); the script's own exit path also publishes zero (Task 4).

---

### Task 0: Branch and baselines

**Files:** none changed.

- [ ] **Step 1:** In the session worktree: `git switch -c op-chestnut-mads-eps-probe c4eedce50a`. Expected: untracked `docs/superpowers/` carried over, HEAD `c4eedce50a`.
- [ ] **Step 2:** Baseline opendbc tests: `opendbc.car.toyota.tests.test_toyota` and `opendbc.safety.tests.test_toyota`. Expected: all pass.
- [ ] **Step 3:** Baseline firmware: build `panda_h7` in the test copy from this tree (see Task 5 command) and compare `main.bin` size with the committed `panda/board/obj/panda_h7/main.bin`. Expected: identical size (proves the toolchain and `PYTHONPATH` are right). Do not commit the result.

### Task 1: opendbc probe flags and helper

**Files:**
- Modify: `opendbc_repo/opendbc/car/toyota/values.py` (flags, `CarControllerParams.__init__`)
- Create: `opendbc_repo/opendbc/car/toyota/eps_probe.py`
- Test: `opendbc_repo/opendbc/car/toyota/tests/test_eps_probe.py`

**Interfaces:**
- Produces: `EPS_PROBE_STEER_MAX: int = 3000`; `eps_probe_active(CP: CarParams) -> bool` (brand `toyota` and `ToyotaFlags.EPS_PROBE` set);
  `with_eps_probe(CI: CarInterfaceBase) -> CarInterfaceBase` (eligible: brand toyota, torque steer control, last safety config `toyota`, not SECOC, not dashcamOnly; sets both flags on `CI.CP` and returns `interfaces[CP.carFingerprint](CP)`; otherwise returns `CI` unchanged).
  `CarControllerParams.STEER_MAX` is 3000 on instances built from a CP with the flag.

- [ ] **Step 1: Write the failing tests** (`TestEpsProbe`, CP from `CarInterface.get_params(CAR.TOYOTA_COROLLA_TSS2, gen_empty_fingerprint(), [], False, False)` via `interfaces`):
  - `test_default_limits`: `CarControllerParams(CP).STEER_MAX == 1500`, `eps_probe_active(CP) is False`.
  - `test_with_eps_probe_sets_both_flags`: new CI is not the old one; `CP.flags & ToyotaFlags.EPS_PROBE`; `safetyConfigs[-1].safetyParam & ToyotaSafetyFlags.EPS_PROBE`; low byte still 73; `CI.CC.params.STEER_MAX == 3000`.
  - `test_angle_platform_not_eligible`: an LTA platform (`ToyotaFlags.ANGLE_CONTROL`, e.g. `CAR.TOYOTA_RAV4_TSS2_2023`) returns the same CI, no flags.
  - `test_other_brand_bit_not_probe`: a non-Toyota CP with `flags |= 4096` gives `eps_probe_active(CP) is False`.
  - `test_controller_reaches_3000`: probe CI, `CI.CS.out.steeringTorqueEps = 5000`, `CarControl(latActive=True, actuators.torque=1.0)`, 300 x `CI.apply(CC, now)`; last `torqueOutputCan == 3000`. Same loop without probe ends at 1500.
- [ ] **Step 2:** Run `opendbc.car.toyota.tests.test_eps_probe`. Expected: FAIL (module missing).
- [ ] **Step 3:** Add the flags to `values.py`, set `self.STEER_MAX = EPS_PROBE_STEER_MAX` in `CarControllerParams.__init__` when the flag is set (define the constant in `values.py`, re-export from `eps_probe.py` to avoid a circular import), write `eps_probe.py`.
- [ ] **Step 4:** Run the new test and `opendbc.car.toyota.tests.test_toyota`. Expected: PASS.
- [ ] **Step 5:** Commit `toyota: EPS probe flags and helper`.

### Task 2: Panda safety limit under the flag

**Files:**
- Modify: `opendbc_repo/opendbc/safety/modes/toyota.h` (globals, `toyota_tx_hook`, `toyota_init`)
- Test: `opendbc_repo/opendbc/safety/tests/test_toyota.py`

**Interfaces:**
- Consumes: `ToyotaSafetyFlags.EPS_PROBE` (Task 1).
- Produces: `static bool toyota_eps_probe`; `max_torque = toyota_eps_probe ? 3000 : 1500`.

- [ ] **Step 1: Write the failing test:** `class TestToyotaEpsProbeSafety(TestToyotaSafetyTorque)` with `MAX_TORQUE_LOOKUP = [0], [3000]` and a param of `EPS_SCALE | ToyotaSafetyFlags.EPS_PROBE` (override the same method the other variants override). It inherits every torque/rate/error/request-cut test at the new maximum. The unchanged `TestToyotaSafetyTorque` keeps proving 1500 without the flag.
- [ ] **Step 2:** Run `opendbc.safety.tests.test_toyota`. Expected: new class FAILs (3000 rejected).
- [ ] **Step 3:** Implement: global `toyota_eps_probe = false`; in `toyota_init`, `#ifdef ALLOW_DEBUG` read `TOYOTA_PARAM_EPS_PROBE = 16UL << TOYOTA_PARAM_OFFSET` next to SECOC, and reset it to false outside the guard first; in the tx hook use it for `.max_torque`.
- [ ] **Step 4:** Run `opendbc.safety.tests.test_toyota`, `opendbc.safety.tests.test_release_build`, and the MISRA check (`opendbc/safety/tests/misra/test_misra.sh`). Expected: PASS, no new MISRA findings.
- [ ] **Step 5:** Commit `safety: Toyota EPS probe torque limit (debug builds only)`.

### Task 3: openpilot wiring

**Files:**
- Modify: `openpilot/selfdrive/car/card.py` (after `get_car`), `openpilot/selfdrive/controls/controlsd.py` (latActive), `openpilot/tools/joystick/joystickd.py` (latActive + `selfdriveStateSP`), `openpilot/system/manager/process_config.py` (drop the on-device `joystick` process)
- Test: `openpilot/tools/op_mads/tests/test_eps_probe_wiring.py`

**Interfaces:**
- Consumes: `with_eps_probe`, `eps_probe_active` (Task 1); `mads_lat_active(selfdrive_state, selfdrive_state_sp)` (existing, `selfdrive/selfdrived/mads.py`).

- [ ] **Step 1: Write the failing source-check tests** (the processes need hardware to import; the fork checks wiring by reading source, see `test_debug_info.py`):
  - `test_card_applies_probe`: `card.py` calls `with_eps_probe(self.CI)` inside a `JoystickDebugMode` check, before `self.RI` is built.
  - `test_controlsd_refuses_probe_session`: the `CC.latActive` expression in `controlsd.py` contains `not eps_probe_active(self.CP)`.
  - `test_joystickd_follows_mads`: `joystickd.py` subscribes `selfdriveStateSP` and its `CC.latActive` uses `mads_lat_active(`.
  - `test_no_device_joystick_publisher`: `process_config.py` has no `joystick_control` process.
- [ ] **Step 2:** `run_tests.sh`. Expected: the four tests FAIL.
- [ ] **Step 3:** Make the four edits.
- [ ] **Step 4:** `run_tests.sh`. Expected: PASS, ruff clean.
- [ ] **Step 5:** Commit `eps probe: card, controlsd, joystickd and manager wiring`.

### Task 4: Probe script

**Files:**
- Create: `openpilot/tools/op_mads/eps_probe.py`
- Test: `openpilot/tools/op_mads/tests/test_eps_probe.py`

**Interfaces:**
- Consumes: `EPS_PROBE_STEER_MAX`, `eps_probe_active` (Task 1).
- Produces (pure, tested): `Step(level: int, target: int, direction: str, duration: float, hold_s: float)`;
  `build_schedule(levels: list[int], hold_s: float = 1.5, rest_s: float = 2.5) -> list[Step]` (rest steps have `target 0`, direction `"rest"`; apply duration = `level / 1500 + hold_s`, the ramp at 15 per 10 ms frame);
  `abort_reason(cs, lat_active: bool, stage: int, lat_lost_s: float) -> str | None`;
  `level_verdict(target: int, cmds: list[float], eps: list[float], faulted: bool) -> str`;
  `probe_ready(CP) -> bool` (= `eps_probe_active`). Positive target = left.

- [ ] **Step 1: Write the failing tests:**
  - `test_default_schedule`: 24 steps, apply targets `[1500, -1500, 1800, -1800, ..., 3000, -3000]` interleaved with rests, total duration 66 s (+-0.01).
  - `test_max_cuts_levels`: `levels_from_args(max=2100)` gives `[1500, 1800, 2100]`.
  - `test_abort_reasons`: each of temp fault, perm fault, `steeringPressed`, rate 101, angle -451, stage 1 at 0.31 m/s, stage 2 at 5.1 km/h, `lat_lost_s` 0.51 returns a non-empty reason; nominal returns `None`; stage 2 at 2 km/h returns `None`.
  - `test_verdicts`: target 2100, cmds 2100, eps 3045 -> `ACCEPT`; target 3000, cmds 2550, eps 2200 -> `CLIP`; eps all 20 -> `NO-RESPONSE`; faulted -> `FAULT`.
  - `test_refuses_without_probe`: `probe_ready` false on a plain Corolla CP, true after `with_eps_probe`.
- [ ] **Step 2:** `run_tests.sh`. Expected: FAIL (module missing).
- [ ] **Step 3:** Implement the pure functions and `main()`: argparse `--stage {1,2}` (required), `--levels`, `--max`, `--dry-run` (prints the schedule and exits); refuses unless `probe_ready(CarParams)`; `SubMaster(['carState', 'carControl', 'carOutput'])`, `PubMaster(['testJoystick'])` at 100 Hz (`axes = [0., target / EPS_PROBE_STEER_MAX]`); waits up to 3 s for `latActive`; collects hold samples from `carOutput.actuatorsOutput.torqueOutputCan` and `carState.steeringTorqueEps`; prints one verdict line per apply step; writes JSONL events (`start`, `step`, `verdict`, `abort`, `end`) with `logMonoTime`; publishes zero in a `finally` block. `--help` carries the in-car procedure from the spec.
- [ ] **Step 4:** `run_tests.sh`. Expected: PASS, ruff clean.
- [ ] **Step 5:** Commit `tools: eps_probe.py step profile with aborts and verdicts`.

### Task 5: Panda firmware and branch docs

**Files:**
- Modify: `panda/board/obj/*` (rebuilt, `git add -u`), `CLAUDE.md` (probe branch section)
- Add: `docs/superpowers/specs/...`, `docs/superpowers/plans/...`

- [ ] **Step 1:** Build `panda_h7` in the test copy with `PYTHONPATH=<session worktree>/opendbc_repo` (same command as Task 0 Step 3), copy the outputs into `panda/board/obj/`. Expected: `main.bin` larger than the baseline, `strings` shows a new `DEV-...-DEBUG` version.
- [ ] **Step 2:** Add a `CLAUDE.md` section like the rate25 one: what the branch is, closed areas only, the in-car procedure, switch and revert commands (push to `fork`, device checkout, reboot, `connected, version` check), never change `JoystickDebugMode` onroad.
- [ ] **Step 3:** Commit firmware, then docs (spec, plan, CLAUDE.md).

### Task 6: Final checks and cleanup

- [ ] **Step 1:** Full `run_tests.sh`, `opendbc.safety.tests.test_toyota`, `opendbc.car.toyota.tests.test_toyota`, `test_eps_probe`. Expected: all PASS.
- [ ] **Step 2:** Restore the shared test copy: for every file this branch changed, copy it back from `.worktrees/op-chestnut-mads` (or delete it if new). Expected: `cmp` clean for those files.
- [ ] **Step 3:** Whole-branch review by a fresh reviewer against the spec; fix findings.
