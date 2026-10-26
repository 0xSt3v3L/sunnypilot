# Toyota EPS probe above 1500: closed-area test design

Date: 2026-10-26. Car: 2024 Taiwan Corolla Altis Hybrid (`TOYOTA_COROLLA_TSS2`, torque steering), comma four on
`op-chestnut-mads` (openpilot v0.11.3 base + MADS; worktree `/Users/steve/openpilot_repos/openpilot/.worktrees/op-chestnut-mads`,
device commit `c4eedce50a`). The implementation is on branch `op-chestnut-mads-eps-probe` from that commit.

## Why

Tight Taiwan right turns at about 16 km/h run wide under openpilot. An offline study of routes a2-cf (13.4 h, 174 low-speed
right turns; scripts in the session scratchpad `eps/`, summary in memory `low-speed-eps-capability`) found:

- The EPS follows the LKA torque command linearly and without measurable lag up to the current 1500 limit
  (`steeringTorqueEps` = 1.45 x command at every level and speed). The EPS is not the bottleneck up to 1500.
- 1500 is not enough. A typical right turn needs about 283 deg of steering wheel (p10 184 deg); with op alone at 1500 the
  wheel holds a median of 187 / 161 / 124 deg at 5-10 / 10-15 / 15-20 km/h. A faster torque ramp alone (the unused
  `op-chestnut-mads-rate25` branch) cannot close this gap.
- Rough extrapolation: about 2000-2500 is needed at 15-20 km/h. Uncertain.

Unknown, and the only question this test answers: **does this car's EPS accept LKA torque commands above 1500?**
Three possible outcomes, all useful:

1. Accepts: `steeringTorqueEps` keeps rising at about 1.45 x command.
2. Clips silently: `steeringTorqueEps` flattens near 2200 (1.45 x 1500) while the command rises. The existing 350
   command-vs-EPS limit then also caps the command at EPS + 350, which is the same signature.
3. Faults: `EPS_STATUS.LKA_STATE` leaves its normal value (temporary 9/11/21/25, permanent 3/17).

The longer-term goal (not this test) is a low-speed torque profile for the user's own car, later possibly for friends with
the same car, similar to the HKG CAN-FD fork (`TonyBinheWu/sunnypilot:hkg-enhanced`: 409 vs 270 below 47 km/h, opt-in).

## Goals

- Measure the EPS response to commands from 1500 to 3000 in a closed area, with the driver in the seat.
- Keep the car's normal driving unchanged on the test branch: the raised limit exists only while probe mode is on, so the
  user can drive to and from the venue on the same build.
- Let the user run the test from an SSH session without Wi-Fi to the Mac, and let the analysis happen later from the rlogs.

## Non-goals

- No change to normal openpilot lateral control, rate limits, or the 1500 limit in daily driving.
- No tuning, no low-speed profile, no rate change. Those come after the result.
- No upstream PR.

## Anti-goals (failure even if it "works")

- The raised limit is reachable outside probe mode (a bug that leaves 3000 active on the road).
- A panda firmware that accepts > 1500 without the probe flag.
- Any removal of existing protections: the car controller's driver torque cut (`MAX_USER_TORQUE` 500; Toyota's panda
  safety is `TorqueMotorLimited` and has no driver torque check of its own), the 350 command-vs-EPS limit (panda and
  car controller), rate limits 15/25, the 100 deg/s steer-rate fault avoidance.
- Normal lateral control (`controlsd`) running with the raised limit. `manager` switches between `controlsd` and
  `joystickd` live on `JoystickDebugMode` (`process_config.py:100-101`) while the car flags are read once at init, so
  clearing the param onroad would hand 3000 to `controlsd`. Guard (my call): when the probe flag is set, `controlsd`
  keeps `CC.latActive` false for the whole onroad session; and the procedure says never to change the param onroad.

## Constraints

- In the car the comma four has no Wi-Fi to the Mac. The user connects to the device by SSH and runs commands by hand.
  Everything must be deployed, built and flashed at home beforehand.
- Safety model: the panda keeps enforcing all limits, only `max_torque` is raised, and only when a new safety param
  flag is set. The flag exists only in `ALLOW_DEBUG` builds (like `TOYOTA_PARAM_SECOC`); the device already runs
  debug-signed panda firmware built on the Mac.
- Code in the fork follows the fork's CLAUDE.md (tests in the test copy, safety C is MISRA clean, `deploy.py` flow).

## How (decided)

### Engagement: MADS lateral, not ACC

`joystickd` (openpilot's joystick debug mode, `JoystickDebugMode`) has no standstill check, unlike `controlsd`
(`steerAtStandstill` is False for Toyota, so normal op never steers below 0.3 m/s; the logs show `latActive` 0% below
1 km/h). But stock `joystickd` gates on `selfdriveState.active`, which needs ACC engaged.

My call: on the test branch, `joystickd` uses `mads_lat_active()` like `controlsd`. MADS stays active while braking in D
at a standstill (it pauses only on wrong gear below 2.5 m/s, reverse, brake hold, park brake, door, seatbelt), and the
panda's MADS rules ignore brake and gas. So the test runs in D with the foot on the brake and ACC main on, no ACC
engagement and no op longitudinal. `CC.longActive` stays false.

### Raised limit only in probe mode

- `JoystickDebugMode` is `CLEAR_ON_MANAGER_START | CLEAR_ON_OFFROAD_TRANSITION`: it is set by SSH while offroad, survives
  into the next onroad session, and clears itself when the car turns off. Probe mode = this param.
- At car init (before the CarController is built), when probe mode is on: set a new `ToyotaFlags` bit on `CarParams` and
  a new `ToyotaSafetyFlags` bit (`16 << 8`, the next free one after SECOC) on the Toyota safety config. Both are set
  together or not at all.
- The CarController and its params are built inside `get_car()` (`opendbc/car/interfaces.py:112`), before `card.py`
  first sees CP, and opendbc cannot read `Params`. So the probe state is either passed into `get_car` or the controller
  is rebuilt after card sets the flags; builder's choice.
- The flags do not carry over to the next drive: `CarParamsCache` only reuses carFw and VIN, card is onroad-only, and the
  param clears on the onroad-to-offroad transition (checked in code).
- `CarControllerParams`: `STEER_MAX = 3000` when the flag is set, else 1500. Rates stay 15/25, `STEER_ERROR_MAX` 350.
  `actuatorsOutput.torque` is normalized by `STEER_MAX`, so it becomes a fraction of 3000 in probe mode; the analysis uses
  the raw `torqueOutputCan`.
- `toyota.h`: `max_torque = 3000` when the flag is set, else 1500. `max_rt_delta` 450 is enough (15 x 25 frames = 375).
  Nothing else in the limits changes.

### Probe script

A new script on the test branch (e.g. `openpilot/tools/op_mads/eps_probe.py`), run by the user over SSH. It publishes
`testJoystick` (axis 1 = torque fraction of `STEER_MAX` 3000) and replaces the on-device `joystick` process, which must not
also publish (disable it on the test branch or have the probe script be that process; builder's choice).

Profile (my call), per level 1500, 1800, 2100, 2400, 2700, 3000, first left then right:
ramp at the normal 15 per frame (the car controller does the ramp; the script just sets the target), hold 1.5 s,
target 0, wait 2.5 s. About 75 s for the whole run. A `--max` argument stops at a lower level, a `--levels` argument
allows a custom list, `--dry-run` prints the schedule.

Abort (target 0 at once, print the reason, exit) on any of:
- `steerFaultTemporary` or `steerFaultPermanent`
- `steeringPressed`
- `|steeringRateDeg| > 100`
- `|steeringAngleDeg| > 450`
- `vEgo` above the stage limit (stage 1: 0.3 m/s; stage 2: 5 km/h)
- `latActive` false for more than 0.5 s after the run starts (MADS paused or disengaged)
- Ctrl-C

Faults: opendbc counts `LKA_STATE` 0 as a temporary fault too (catch-all, `carstate.py:16-18`), and `joystickd` drops
`latActive` on any fault. If `latActive` never comes up within 3 s of the start, the script exits with "no lateral at
this speed" instead of running the profile.

After each level the script prints a one-line verdict so the user can decide on stage 2 in the car (my call): hold-time
median EPS / command ratio, whether the command reached the target, and any `LKA_STATE` change. Verdict per level:
`ACCEPT` (command reached target and ratio >= 1.2), `CLIP` (command short of target by > 100, or ratio < 1.2 with no
fault), `FAULT`, `NO-RESPONSE` (EPS < 100 for the whole hold).

The script writes a small JSON lines file to `/data/eps_probe/<route>--<time>.jsonl`: run start, each step's target and
start/end monoTime, the abort reason if any. The analysis aligns it with the rlog.

### Stages

- **Stage 1, standstill.** D, foot on the brake, ACC main on (MADS active), wheels straight. `eps_probe.py --stage 1`.
  The tires scrub, so the wheel barely moves. This answers the question directly. If the EPS ignores LKA torque at
  0 km/h (no data either way: op never steered below 1 km/h), stage 1 shows `steeringTorqueEps` near 0 for every level,
  including 1500. That itself is a result; go to stage 2.
  If stage 1 faults already at 1500 (user's decision, 2026-10-26): treat it as "the EPS rejects LKA at standstill", not as
  a verdict on > 1500, because the logs show the EPS responding at 1500 at 1-2 km/h. Turn the car off and on, confirm
  `LKA_STATE` is back to normal (no fault on the onroad screen, MADS active), then go to stage 2. If the fault does not
  clear after the ignition cycle, stop for the day.
- **Stage 2, creeping at 1-3 km/h.** Only if stage 1 shows outcome 1, the EPS ignores standstill commands (no lateral or
  `NO-RESPONSE`), or stage 1 faulted at 1500 and the fault cleared. Not after a stage 1 fault above 1500. Driver holds
  1-3 km/h with the brake in a clear, flat area, straight ahead. `eps_probe.py --stage 2`. The EPS is known to respond at
  1-2 km/h (1.45 x, up to 1500 in the logs). The wheel will turn: the angle and rate aborts matter here.
- Rolling tests at 5-10 km/h that measure the held angle above 1500 are a later, separate test.

### In-car procedure (to be printed in the test branch's CLAUDE.md and as `eps_probe.py --help`)

1. At home: deploy the test branch, reboot, confirm the panda flashed (`panda firmware out of date` and
   `done flashing panda` in swaglog) and the device is healthy. Drive to the venue normally (probe mode is off: stock 1500).
2. At the venue, car off (device offroad): SSH in, `echo -n 1 > /data/params/d/JoystickDebugMode`.
3. Car on (READY), ACC main on, D, foot on the brake. Check the onroad screen shows the joystick alert and MADS active.
4. Run stage 1. Keep hands near (not on) the wheel, foot on the brake.
5. If stage 2 is warranted: run it.
6. Car off. The param clears itself. Drive home normally.
7. At home: pull the route (`pull_route.sh`) and copy `/data/eps_probe/`. Claude analyzes.

### Analysis (after the test)

From the rlog: command (`carOutput.actuatorsOutput.torqueOutputCan`), `steeringTorqueEps`, raw `EPS_STATUS.LKA_STATE`
from `can`, `steeringAngleDeg`, `steeringRateDeg`, `vEgo`, driver torque, `latActive`, plus the probe JSONL for step
timing. Per level and direction: the steady EPS/command ratio during the hold, whether the command reached the target,
LKA_STATE changes, wheel movement. Reuse `eps/extract.py` from the scratchpad (add LKA_STATE from `can`).

### Deploy

`deploy.py --branch` switches branches but does not build panda firmware: build it on the Mac per the fork's CLAUDE.md
(with `PYTHONPATH=<tree>/opendbc_repo`, or it silently compiles another tree's safety) and commit it with the branch.

### Rollback

Switch the device back to `op-chestnut-mads` with `deploy.py` and reboot, so the panda flashes the normal firmware. A
permanent EPS fault (LKA_STATE 3 or 17) clears with an ignition cycle according to opendbc's comments. Whether it also
leaves a stored DTC that needs a dealer tool is unknown; this is the main risk the user accepts.

## Left to the builder

- Branch name and worktree (suggest `op-chestnut-mads-eps-probe` from `c4eedce50a`).
- Where exactly the probe flags are set at init (card / interface), as long as both flags come from the same check before
  the CarController exists.
- How the on-device `joystick` process is kept from publishing.
- Tests: safety tests for the new flag (1500 without it, 3000 with it, rate/error limits unchanged, flag absent in
  non-debug builds), a car-controller test for `STEER_MAX`, a unit test for the probe schedule and abort checks, a test
  that `joystickd` follows MADS.
