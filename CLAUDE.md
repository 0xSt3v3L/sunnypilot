# op-chestnut-mads

A personal fork of openpilot v0.11.2 (build `4800490a7`) for one car: a 2024 Taiwan Corolla Altis Hybrid
(`TOYOTA_COROLLA_TSS2`, torque steering, openpilot longitudinal) with a comma four and a chestnut eGPU.
The branch is pushed to the git remote `fork`; `origin` is commaai.

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
| Low-speed lateral deadband (no wheel turn before a stop) | `selfdrive/controls/lib/low_speed_lat.py`, `modeld.py` |
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

- The copy lives next to the worktree (`../op-mads-test`, override with `OP_MADS_TEST`) and is synced by
  `../op-mads-sync.sh` (`OP_MADS_SYNC`), which copies the files changed since `577b3b55f`. A file reverted back to that
  base stays stale in the copy; check with `cmp`.
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
- The device's big driving model is commaai `6f4c0faf1f` "Mountain Dew Model (#39047)", checkpoint `.../8d736683.../12864`,
  shipped only as a `USB+AMD:LLVM` pkl. comma did not publish its ONNX; it was rebuilt from the device pkl's weights on
  top of the public `870a4823` export and checked against logged `modelV2` by replay. The files, METAL builds (about
  30 ms a frame) and how they were made live next to the logs in `op-mads-models/` (`OP_MADS_MODELS`, see its README).
  A new big model on the device means rebuilding that ONNX.

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
- Process replay defaults `DisengageOnAccelerator` to True; the device has it off, so `mads_replay.py` replays with False
  (otherwise every gas press looks like a disengagement).
- Commit messages: English, imperative subsystem prefix (`mads:`, `ui:`, `controls:`), ending with the
  `Co-Authored-By` trailer.
