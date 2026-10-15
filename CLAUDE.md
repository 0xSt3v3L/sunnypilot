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
| Low-speed lateral deadband (no wheel turn before a stop) and swing shaping (7-22 km/h) | `selfdrive/controls/lib/low_speed_lat.py`, `modeld.py` |
| Toyota resume race fix (RES while braking latched a cruise fault) | safety `opendbc_repo/opendbc/safety/modes/toyota.h`; firmware `panda/board/obj` |
| MADS process replay | `selfdrive/test/process_replay/mads_replay.py` |
| Navigation with Google routes and comma's nav model (`NavigationEnabled`, default off; branch `op-nav-google` only) | `selfdrive/navigation/` (navd, mapsd, renderer and tracking from comma's nav-model-integration; `google.py`, `navweb.py`), `modeld.py` model switch and nav inputs, mici navigation panel and developer toggle, `tools/op_mads/nav_model_replay.py` |
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

## Navigation on the device (branch `op-nav-google`)

- Deploy from the `op-nav-google` worktree with `deploy.py --branch op-nav-google`; the device checks the branch out in
  the build command, with the branch's own committed binaries. Going back: `deploy.py` from the op-chestnut-mads worktree.
  The branch adds params and cereal services, so its first deploy rebuilds libparams, cereal, pandad and loggerd.
- Files the device needs, put by hand (not in git): `/data/navigation/google_maps_api_key` (Routes API and Places API
  (New) enabled; `chmod 600`) and `/data/navigation/basemap.pmtiles`, a zoom-15 Protomaps extract of Taiwan
  (`pmtiles extract https://build.protomaps.com/<yyyymmdd>.pmtiles basemap.pmtiles --bbox=119.3,21.8,122.1,25.4
  --minzoom=15 --maxzoom=15`, about 154 MB; the Mac copy is in `.worktrees/nav-basemap/`). Without the basemap the map
  input stays black; without the key there is no route or search.
- `big_driving_nav_tinygrad.pkl` is a git-lfs object (822 MB, on comma's HuggingFace LFS). modeld uses it only when it
  is more than a pointer on disk, else it keeps the stock model and logs an error; check its size after a deploy.
- `NavigationEnabled` (developer settings, offroad only) runs navd and mapsd onroad and navweb always, and makes modeld
  load the nav model at its next start. Off goes back to the stock model on the next drive, no deploy needed.
- The phone shares its hotspot; the device joins it as Wi-Fi. Developer settings > navigation page shows a QR code of
  `http://<wlan0 address>:8099/?t=<token>` (token in `/data/navigation/web_token`); the page searches places, takes a
  Google Maps share link or coordinates, and sets or clears `NavDestination` (cleared at each manager start and offroad
  transition, so set it after the device is up).
- navd routes only from a fix of 5 m or better and reroutes after more than 3 s over 25 m off the route.

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
  The low-speed lateral shaping postdates routes up to 000000a5, so their logged desiredCurvature no longer matches a
  replay (rmse/std about 1.1 on a5/2); the model outputs themselves still do.
- `nav_model_replay.py <route dir> --segments 5-9 --model big_nav_afbf2e5_metal.pkl --tiles <PMTiles URL> [--maps out.png] [--ablation]`:
  runs comma's nav model twice over the same frames, without and with the map and navigation inputs, from a Google route
  along the driven GPS track, and prints the curvature (+ = right) around each turn maneuver. Google key from
  `GOOGLE_MAPS_API_KEY` or `~/.config/op-mads/google_maps_api_key`; `--tiles` is a public Protomaps build
  (`https://build.protomaps.com/<yyyymmdd>.pmtiles`, list at `build-metadata.protomaps.dev/builds.json`), read by range
  and cached in `~/.comma/navigation/tiles`. Same environment as `big_model_replay.py`.
  - Vias go 40 m past each driven turn with the direction of travel (and every 400 m on long stretches, at most 25):
    dense or headingless vias make Google detour into alleys and U-turn. Like navd, the replay reroutes after more than
    3 s over 25 m off the route; a new route's progress starts at 0 so a loop back past its start cannot capture it.
  - GPS after a cold start can sit 30-40 m off the road for minutes (a5 until about 300 s, `horizontalAccuracy` about
    10 m); later fixes are within a few meters. Pick segments after that.
  - Model files: the branch's ONNX is "rl nav driving model" (checkpoint 80eedcdd), in `op-mads-models/nav_afbf2e5/`,
    compiled with `tinygrad_repo/examples/openpilot/compile_onnx.py --out-of-band` (`DEV=METAL JIT=2`). The newer n16
    nav-wm model exists only as the device pkl. Its inputs: `nav_map` float32 (1, 1, 256, 256) fed the 0-255 map,
    `navigation` (1, 47) = Valhalla command one-hot + distance / 100. openpilot's LFS objects live on
    `huggingface.co/commaai/openpilot-lfs` (see `.lfsconfig`): fetch one through its LFS batch API by oid and size.
  - comma's nav endpoints (`v1/<dongle>/nav/tiles`, `v1/nav/directions`) answer 403 to this device's valid JWT.
  - Ablation over 12 real turns (a2 5-7, a3 18-21, a4 14-16, a5 5-9; rl nav 80eedcdd): both inputs move the mean
    curvature toward the turn in 11/12, median +15% (+0.005 1/m); the map alone +12%, the instruction alone +6%; the
    mirrored instruction gives less than the right one in 12/12 but never turns the other way. The replay is open loop:
    the video already shows the driver's turn, so it cannot show whether the model would start a turn on its own; a
    wrong left at a5 316 s (the driver went straight) moved nothing. Only a closed-loop drive can answer that.

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
- The 7-22 km/h left-right wheel swing (~1.3 s period) is in the model's request, not the torque controller: the
  request (as wheel angle) swings as much as the wheel, the wheel 0.4 s behind, and a closed-loop sim with the real
  controller is stable with a zero request. Controller changes (lower KP, a D term) only shrink it. `LowSpeedLatShaper`
  scales the fast part of small requests at 2-6 m/s (steady bends and turns untouched).
- Process replay defaults `DisengageOnAccelerator` to True; the device has it off, so `mads_replay.py` replays with False
  (otherwise every gas press looks like a disengagement).
- Commit messages: English, imperative subsystem prefix (`mads:`, `ui:`, `controls:`), ending with the
  `Co-Authored-By` trailer.
