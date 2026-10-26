# op-chestnut-mads

A personal fork of openpilot for one car: a 2024 Taiwan Corolla Altis Hybrid (`TOYOTA_COROLLA_TSS2`, torque steering,
openpilot longitudinal) with a comma four and a chestnut eGPU. The branch is pushed to the git remote `fork`; `origin` is
commaai.

Upstream base: openpilot v0.11.3 nightly-chestnut `ecec603414`, master `2ab952d5eb` (MD v3 big model), merged on
2026-10-26 (the previous merge was master `75b2ccca60` on 2026-10-16). Upstream comes in as release builds (vendored
submodules, prebuilt binaries), which share no history with master: the fork started from the v0.11.2 build
`4800490a7`; `a6f7f353b3` (75b2ccca60) and `f5715f8e81` (2ab952d5eb) are `nightly-chestnut` build trees each committed
with the previous upstream build as its parent (`git commit-tree origin/nightly-chestnut^{tree} -p <last upstream
commit>`), then merged. Do the same for the next upstream: conflicts in the fork's own binaries (libparams_c.so,
pandad, loggerd, cereal/gen, panda_h7) keep ours and get rebuilt; every other prebuilt binary comes from the build.
Check `git_src_commit` of the build: master moves ahead of it (AGNOS 20.1 and later are on master only as of
2026-10-26). Upstream's "A New Dawn" removed the ExperimentalMode param: openpilot longitudinal always runs the
experimental (e2e) planner; `selfdriveState.experimentalMode` is deprecated upstream, but selfdrived still writes
`deprecated.experimentalMode` for the log tools.

## Test branch op-chestnut-mads-rate24-probe: closed areas only

`op-chestnut-mads-rate24` merged with `op-chestnut-mads-eps-probe` (both described below), to check in a closed area
whether the EPS follows the 24-per-frame ramp: `eps_probe.py` steps 0 -> 1500 and the car controller ramps at the low-speed
rate. The question is whether the EPS has an internal LKA rate limit like the ~1500 units/s noted for LTA (that would
make rate24 useless) and whether a faster ramp faults it.

In the car (standstill is enough; stage 2 creeping is optional, real low-speed turns on rate24 cover motion later):
1. Car off, device offroad: `echo -n 1 > /data/params/d/JoystickDebugMode`. Never change it while onroad.
2. Car on (READY), ACC main on, D, foot on the brake, wheels straight.
3. In tmux, three times: `cd /data/openpilot && python3 openpilot/tools/op_mads/eps_probe.py --stage 1 --levels 1500`.
   Always pass `--levels 1500`: the default levels go up to 3000, and the EPS faults above 1500 (route 000000d1).
   Expect ACCEPT left and right; ignore the "stage 2" line (it is written for the above-1500 test).
4. Car off, then pull the route and `/data/eps_probe/`.

Pass: below 30 km/h the CAN command rises 24 per frame, `steeringTorqueEps` keeps about 1.45 x the command during the
ramp (it would fall behind if the EPS limited the rate to 15), no `LKA_STATE` / `steerFaultTemporary` fault, and
`safetyTxBlocked` stays 0. Switch and revert like the branches below, with branch `op-chestnut-mads-rate24-probe`.

## Validation branch op-chestnut-mads-rate24: closed areas first

`op-chestnut-mads` (`fbacb5398c`) plus a faster LKA torque ramp at low speed: 24 per frame up to 30 km/h, the stock 15 from
40 km/h (`low_speed_steer_delta_up` in the Toyota car controller; the panda safety allows the same curve read 1 m/s low,
with the real-time limit at 30 frames of the rate: 720 / 450). 24, not 25, keeps the 0.5 s jerk within the ISO 11270
limit of `test_lateral_limits`. It cannot help above 1500: the EPS faults on any LKA command above 1500 (closed-area
probe, route 000000d1, 2026-10-26). Merge into `op-chestnut-mads` only after the closed-area check passes and the user
agrees.

Closed-area check: low-speed turns at 10-25 km/h in an empty area with MADS, both directions, then the rlog must show
the command rising 24 per frame below 30 km/h, no `steerFaultTemporary` / `LKA_STATE` fault, and `safetyTxBlocked`
staying 0.

Switch the device (offroad; push and reboot only with the user's approval). Run device commands in a login shell
(`bash -lc`) when they need python:

```bash
git push fork op-chestnut-mads-rate24
ssh comma_4 'set -e; cd /data/openpilot; test "$(cat /data/params/d/IsOnroad 2>/dev/null)" != 1; git checkout -q -- panda/board/obj/gitversion.h panda/board/obj/version; git fetch -q origin op-chestnut-mads-rate24; git checkout -q -B op-chestnut-mads-rate24 FETCH_HEAD; git log --oneline -1'
```

Back to the normal build (offroad, then reboot; pandad flashes the normal firmware back):

```bash
ssh comma_4 'set -e; cd /data/openpilot; test "$(cat /data/params/d/IsOnroad 2>/dev/null)" != 1; git checkout -q -- panda/board/obj/gitversion.h panda/board/obj/version; git checkout -q op-chestnut-mads; git log --oneline -1'
```

After each reboot: `ssh comma_4 'grep -h "firmware out of date\|done flashing panda" /data/log/swaglog.* | tail -3'`
(the version string stays `DEV-6f4c0faf-DEBUG`; pandad's post-flash signature check failing would stop pandad).

## Test branch op-chestnut-mads-eps-probe: closed areas only

This branch is `op-chestnut-mads` (`c4eedce50a`) plus an EPS probe: in joystick debug mode the Toyota LKA torque limit is
3000 instead of 1500, in the car controller and in the panda (`ToyotaFlags.EPS_PROBE` / `ToyotaSafetyFlags.EPS_PROBE`,
set together by `opendbc/car/toyota/eps_probe.py` from card at init; the panda reads its flag only in debug builds).
Design and plan: `docs/superpowers/specs/2026-10-26-toyota-eps-above-1500-probe-design.md`,
`docs/superpowers/plans/2026-10-26-toyota-eps-probe.md`.

- Normal driving on this branch is unchanged: without `JoystickDebugMode` the limit is 1500 everywhere.
- In a probe drive `controlsd` never steers, `joystickd` follows MADS lateral (ACC main is enough; no standstill check),
  and `openpilot/tools/op_mads/eps_probe.py` is the only `testJoystick` publisher. Its `--help` is the in-car procedure.
- Never change `JoystickDebugMode` while onroad. It clears itself on the onroad-to-offroad transition.
- The firmware keeps the version string `DEV-6f4c0faf-DEBUG` (it comes from the build copy's git HEAD), so after the
  reboot that follows a switch, check that pandad flashed: swaglog shows `panda firmware out of date` and
  `done flashing panda` once each. If the panda still blocks torque above 1500, `eps_probe.py` aborts with
  "panda blocked steering" instead of reporting an EPS fault.
- Run `eps_probe.py` inside `tmux` (a dropped SSH session does not stop it; SIGHUP and SIGTERM stop it cleanly). To stop
  a run at once without SSH: ACC main off (MADS drops lateral) or hold the wheel (driver steering aborts it).
- Pre-flight at home after the switch: `eps_probe.py --stage 1` on a normal drive must print "probe mode is not active"
  (this checks the imports and params on the device; `--dry-run` imports nothing). Passed on 2026-10-26 at `ebeda7b`.
- Run device commands in a login shell: an interactive `ssh comma_4` is one; for `ssh comma_4 '<cmd>'` use
  `bash -lc "<cmd>"`, otherwise `python3` is `/usr/bin/python3` without capnp.

Switch the device to the probe (offroad; push and reboot only with the user's approval):

```bash
git push fork op-chestnut-mads-eps-probe
ssh comma_4 'set -e; cd /data/openpilot; test "$(cat /data/params/d/IsOnroad 2>/dev/null)" != 1; git checkout -q -- panda/board/obj/gitversion.h panda/board/obj/version; git fetch -q origin op-chestnut-mads-eps-probe; git checkout -q -B op-chestnut-mads-eps-probe FETCH_HEAD; git log --oneline -1'
```

Back to the normal build (offroad, then reboot; pandad flashes the normal firmware back):

```bash
ssh comma_4 'set -e; cd /data/openpilot; test "$(cat /data/params/d/IsOnroad 2>/dev/null)" != 1; git checkout -q -- panda/board/obj/gitversion.h panda/board/obj/version; git checkout -q op-chestnut-mads; git log --oneline -1'
```

After each reboot: `ssh comma_4 'grep -h "firmware out of date\|done flashing panda" /data/log/swaglog.* | tail -3'`.
After the test: pull the route with `pull_route.sh` and copy `/data/eps_probe/` from the device.

## What this branch adds

Most features are ported from sunnypilot (MIT; keep a local checkout to compare against) and hard-coded for this car.

| Feature | Where |
|---|---|
| MADS: lateral-only engagement, Toyota only | safety `opendbc_repo/opendbc/safety/mads.h`, `modes/toyota.h`; firmware `panda/board/`; `selfdrive/selfdrived/mads.py`; pandad heartbeat |
| Road edge lane change block | `selfdrive/controls/lib/relc.py`, published in `modelDataSP.roadEdge` |
| Green light / lead departure alerts | `selfdrive/selfdrived/standstill_alerts.py` |
| Onroad debug overlay (ui debug mode) | `selfdrive/ui/lib/debug_info.py`, `selfdrive/ui/mici/onroad/debug_overlay.py` |
| Always offroad (`OffroadMode`) | `system/hardware/hardwared.py`, `selfdrive/ui/lib/always_offroad.py`, mici settings |
| Low-speed turn desire (`LaneTurnDesire`, default off) | `selfdrive/controls/lib/lane_turn.py`, `desire_helper.py`, `modeld.py` |
| Low-speed lateral deadband (no wheel turn before a stop) and swing shaping (7-22 km/h) | `selfdrive/controls/lib/low_speed_lat.py`, `modeld.py` |
| Toyota resume race fix (RES while braking latched a cruise fault) | safety `opendbc_repo/opendbc/safety/modes/toyota.h`; firmware `panda/board/obj` |
| MADS process replay | `selfdrive/test/process_replay/mads_replay.py` |
| Dev tools | `openpilot/tools/op_mads/` (deploy, tests, drive report, problem map, frames, route pull) |

Paths in the table are under `openpilot/` unless they start with `opendbc_repo/` or `panda/`.

## Committed binaries: the rule that breaks the car

The device runs prebuilt (`prebuilt` file present, launch does not run scons). These are committed and must be rebuilt
ON THE DEVICE when their inputs change:

| Change | Rebuild |
|---|---|
| `openpilot/common/params_keys.h` (new param) | `openpilot/common/libparams_c.so` |
| `openpilot/cereal/*.capnp`, `cereal/services.py` | `cereal/gen/cpp/*` (via pandad/loggerd builds), `pandad`, `loggerd` |
| panda firmware sources | `panda/board/obj/*` (built on the Mac with the 13.2 arm toolchain in the test copy venv, debug-signed) |

- Python's `Params` validates keys against `libparams_c.so`. New Python that reads a new param with the old lib raises
  `UnknownKeyName`; in hardwared that keeps the device from going onroad. Never let the device hold new code with old
  binaries: merge and build in one ssh command. `openpilot/tools/op_mads/deploy.py` does exactly this; use it.
- Do not push commits that add params or schema changes until the matching device build is committed with them.
- `cereal/gen` and `panda/board/obj` are gitignored but tracked: stage them with `git add -u <paths>`.
- After a device build, `panda/board/obj/gitversion.h` and `version` show as modified on the device. Never commit them:
  they must keep matching the committed firmware.
- Regenerate `services.h` on the Mac when services change: `python3 openpilot/cereal/services.py > openpilot/cereal/services.h`.
- Cereal slots used by this fork: `Event.selfdriveStateSP @107` (custom `SelfdriveStateSP`), `Event.modelDataSP @108`
  (custom `ModelDataSP`), `OnroadEvent.EventName` @105-@120, `PandaState.controlsAllowedLateral @38`.

## Testing

The worktree has no Python env. Tests run in a macOS test copy with a uv venv and macOS natives:

```bash
openpilot/tools/op_mads/run_tests.sh   # sync, rebuild libparams_c.dylib if needed, fork tests + core suites, ruff
```

- The copy lives next to the worktree (`../op-mads-test-up1019`, override with `OP_MADS_TEST`; `../op-mads-test-v0113`
  is the copy of the previous upstream, `../op-mads-test` the v0.11.2 one) and is synced by `../op-mads-sync.sh` (`OP_MADS_SYNC`), which copies the files changed since
  `577b3b55f`. A file reverted back to that base stays stale in the copy; check with `cmp`. The sync skips a Linux (ELF)
  binary from the release tree where the copy has a macOS native.
- A copied test copy keeps the old copy's absolute paths in `.venv` (editable `.pth` finders, script shebangs), so
  `import opendbc` or `tinygrad` silently resolves to the old copy: rewrite them (`grep -rlI` in `.venv/bin` and
  `.venv/lib`) after copying.
- tinygrad on the Mac needs `HCQ_RUNTIME_DEV=CPU:CLANG` (there is no libLLVM for its default host runtime) to compile
  or run models.
- Single modules: `cd <copy> && .venv/bin/python -m unittest openpilot.selfdrive...test_x`. Lint: `.venv/bin/ruff check`.
- New param keys need the copy's `libparams_c.dylib` rebuilt (`run_tests.sh` does it).
- opendbc safety tests use a separate env:
  `cd opendbc_repo && PYTHONPATH=$PWD <opendbc env>/.venv/bin/python -m unittest <module>`, where the env is an opendbc
  checkout at the fork's opendbc base with its own venv (kept next to the worktree as `../opendbc-env`).
  Safety C follows MISRA: every `if/else if` has an `else`, unsigned constants take `U`, no implicit bool/int casts.
- mici UI modules segfault on import without a display. Put UI logic in pure helpers under `selfdrive/ui/lib/` and test
  those; check the rendering wiring with source-reading tests (see `test_debug_info.py`).
- `modeld.py` cannot be imported without its model runtime: same approach, source checks.
- Process replay needs `build.json` at the test copy root (`get_build_metadata` fails in an export without `.git`), and
  scripts that start replays need an `if __name__ == "__main__":` guard (macOS multiprocessing uses spawn).
- Process replay rebuilds CarParams without card, so it must call `set_mads_alternative_experience` (it does); without it
  MADS is unavailable in every replay.
- Use op routes for replays. sunnypilot routes use different MADS semantics in `pandaStates` and replay with false
  `controlsMismatch` events.
- ruff rejects implicitly concatenated strings across lines: join with `+`.

## The comma four

- Reached through an ssh host alias (the tools default to `comma_4`; `pull_route.sh` takes `OP_MADS_DEVICE`). AGNOS 19.9,
  repo at `/data/openpilot` (its `origin` is the fork), `DisableUpdates=1`.
- Deploy only while offroad (`/data/params/d/IsOnroad` not `1`). Rebooting needs the user's explicit approval.
- `deploy.py --dry-run` runs the preflight; `deploy.py --reboot --trailer "Co-Authored-By: ..."` deploys and reboots.
- Device scons is at `/usr/local/venv/bin/scons` (not on the ssh PATH).
- op flashes the chestnut firmware only while offroad; after switching branches the chestnut may need a manual flash.
- The device's big driving model is commaai MD v3 (`1e219d7785`, checkpoint `1365126a.../12864`, LFS oid `d4c7a3fc`). A
  big model pkl only loads with the tinygrad it was compiled with (the Cinque v3 pkl fails on the current one; MD for
  this tinygrad is `21edbbcfb9`, oid `b3864072`). Routes up to 000000b1 ran MD (the v0.11.2 build, `256f2178`),
  000000b2-000000c4 MD v3, 000000c5-000000c8 MD again (2026-10-19, `e9efd7b190`): MD cuts curves less on the car (inside
  offset median 0.39 m vs 0.58 m), but the user preferred how MD v3 drives and it was restored. The big models since
  BMRLNAP v6 were compared offline (`op-mads-logs/model_compare.txt`, scripts in `op-mads-models/compare/`): BMV6 is
  best on curves, no-lead speed, lead pull-away and launches but turns at intersections with about 0.6x MD v3's
  curvature, and needs the pre-#38916 input queues ported; no model brakes for a red light before about 25 m (< -1
  m/s^2). Big model pkls are `USB+AMD:LLVM` only; comma published ONNX for some (HuggingFace
  `commaai/openpilot_driving_models`), MD v3's was rebuilt from its pkl's weights on top of the official MD ONNX
  (differs only in the policy, 92 buffers). The ONNX files, METAL builds for the current tinygrad
  (`op-mads-models/v0113`, about 29 ms a frame) and how they were made live next to the logs in `op-mads-models/`
  (`OP_MADS_MODELS`, see its README, `mdv3/`, `compare/`). A new big model on the device means checking its offline
  replay first.

## Logs and analysis

- `openpilot/tools/op_mads/pull_route.sh --list | <route> [dest]` copies rlogs to `<dest>/<route>/<seg>/rlog.zst`
  (`dest` defaults to `OP_MADS_LOGS`). Keep route data outside the repo (size, GPS positions).
- `drive_report.py <route dir>`: MADS time, standstill alerts and missed stops, lane turns with model curvature, road
  edges, blind spot onsets with speed (low-speed right turns), curves (speed before entry vs model `desiredAcceleration`),
  lateral tracking (torque error by speed, saturation, debounced driver overrides), override causes (lane change,
  blinker turn, curve, low speed, straight; opposing or assisting), braking by openpilot longitudinal classified as
  lead / curve / model stop / unexplained (suspected phantom braking), and chestnut health.
- Report, map and frame times are seconds from the route start (`route_t0`, the first message of the first segment), so
  a time from the report can go straight into `frames.py`.
- `frames.py <route dir> <seconds...> [-o out.jpg]`: captioned front camera frames (speed, commanded accel, lead, model
  plan, longitudinal mode) as one contact sheet. Missing `qcamera.ts` segments are copied from the device; decoding needs
  PyAV in the test copy venv (`uv pip install --python .venv/bin/python av`). Use it to check any report finding on video.
- `problem_map.py <logs dir>`: a Leaflet map of problems across all routes (disengagements, saturation, overrides,
  blocked lane changes, missed alerts, hard model braking in curves). It holds driving positions: keep it next to the
  logs and do not publish it.
- `mads_replay.py <route dir>`: replays selfdrived and compares MADS / engagement transitions with the logged run.
- Run both from the test copy with `PYTHONPATH=<worktree>/opendbc_repo:$PWD .venv/bin/python <script> <route dir>`.
- `big_model_replay.py <route dir> [--segments 2-4] [--mode logged|ab|action_t]`: runs the big model on the Mac (METAL) from
  `fcamera.hevc` / `ecamera.hevc` (copied from the device when missing, about 75 MB a segment) and compares with the
  logged modelV2, plus model curvature at each low-speed blinker turn. `logged` feeds the desire the device fed (rebuilt
  from the previous frame's `laneChangeState` and `laneTurnDirection`); `ab` runs LaneTurnDesire off and on side by side;
  `action_t` adds `--extras` seconds to the model's lateral action time in low-speed blinker turns.
  The first 10 s are model warm-up. Needs `OP_MADS_MODELS`, `DEV=METAL` and `$PWD/tinygrad_repo` on `PYTHONPATH`.
  Encoded video makes the replay inexact: rmse/std about 0.03-0.2 against the log is the floor.

## Conventions and invariants

- Features are hard-coded unless the user asks for a toggle. Each new param costs a device rebuild of
  `libparams_c.so`, so batch new params into one deploy.
- `Mads.update()` runs after the stock `StateMachine.update()`; MADS events carry `ET.ENABLE` and would engage
  openpilot if the stock state machine saw them.
- Lateral follows `mads.active` while MADS is available, also when fully engaged. Do not latch a lateral mismatch to
  keep MADS disabled: a later cruise engage would give an engaged openpilot that never steers (reverted in `a7a3c9a39`).
  A mismatch behaves like stock `controlsMismatch`, and the panda still gates steering.
- The standstill alerts only fire while openpilot does not control longitudinal and after the car first moved.
- Blind spot: op sets `ToyotaFlags.HAS_BSM` only if `0x3F6` shows up during fingerprinting; it was set on every op route
  so far. The owner's manual says BSM works above about 16 km/h, but logs show onsets down to 10-11 km/h (also after
  minutes below 16 km/h) and none below 8 km/h: treat about 9-10 km/h as the real floor for blind spot features.
- Sign conventions, verified on route 000000a5: `steeringAngleDeg` and driver `steeringTorque` are positive to the left;
  model/controls curvature and `desiredLateralAccel` are positive to the RIGHT. Check the sign before comparing them.
- `torqueState.saturated` only turns on after seconds at the limit and stays off through an intersection turn; look at
  `carControl.actuators.torque` reaching +-1 instead (the drive report does).
- Steering torque is the limit at low speed: below 15 km/h the torque command sits at +-1 for 19-39% of openpilot's own
  steering time (2-5% above 40 km/h). Right turns entered above about 10 km/h run wide because the wheel cannot turn
  fast enough, with or without `LaneTurnDesire`; op's 3 m/s^2 lateral accel clip rarely matters. The Toyota torque limit
  is a panda safety limit: do not raise it. The learned latAccelFactor is about 1.40-1.46 against 1.99 in CarParams
  (the car has ET35 wheels since before comma was installed, so there is no before/after data).
- modeld computes curvature as the model's lateral accel / max(1, v)^2, so near a stop a tiny request becomes a large
  curvature. This model asks for +0.015 to +0.022 m/s^2 (right) while stopping, which turned the wheel 20-45 deg right
  before every stop (sunnypilot's own modeld derives curvature differently and did not). `low_speed_lat.py` drops
  requests under 0.03 m/s^2 below 3 m/s, also with a blinker on (with either blinker the stop bias is +0.005 to
  +0.011 m/s^2 and turned the wheel 22-38 deg right); real low-speed turns ask for 0.15+ m/s^2 and are untouched.
  MD v3 halves that bias in offline replays of the same frames (small requests while stopping: median +0.005 vs +0.011
  without a blinker, +0.012 vs +0.022 with one) but does not remove it, so the deadband stays. Same replays: the
  7-22 km/h swing component is unchanged, low-speed turn peaks are about 5% larger, launches behind a moving lead ask
  for about +0.15 m/s^2 more, and a9 726-744 s (not following a lead pulling away) is unchanged
  (`op-mads-logs/mdv3_ab.txt`).
- The 7-22 km/h left-right wheel swing (~1.3 s period) is in the model's request, not the torque controller: the
  request (as wheel angle) swings as much as the wheel, the wheel 0.4 s behind, and a closed-loop sim with the real
  controller is stable with a zero request. Controller changes (lower KP, a D term) only shrink it. `LowSpeedLatShaper`
  scales the fast part of small requests at 2-6 m/s (steady bends and turns untouched).
- Process replay defaults `DisengageOnAccelerator` to True; the device has it off, so `mads_replay.py` replays with False
  (otherwise every gas press looks like a disengagement).
- Commit messages: English, imperative subsystem prefix (`mads:`, `ui:`, `controls:`), ending with the
  `Co-Authored-By` trailer.
