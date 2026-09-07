# Toyota Auto Brake Hold Design

## Status

Approved in chat on 2026-09-07.

## Goal

Port the 2024 Toyota automatic brake-hold behavior to the current sunnypilot `dev` architecture, with a user-controlled setting and revised activation behavior:

- support only camera-ACC Toyota/Lexus TSS2/LSS2 platforms;
- exclude radar-ACC and SecOC platforms;
- begin timing only after the vehicle is at a standstill while the driver is holding the brake pedal;
- activate after 500 ms of continuous brake application at standstill;
- keep brake hold latched when the driver releases or presses the brake pedal again;
- release on accelerator input, vehicle movement, ACC activation, cruise-main deactivation, or shifting into park or reverse.

## Non-goals

- Supporting radar-ACC Toyota platforms.
- Supporting Toyota SecOC platforms.
- Changing Toyota longitudinal tuning.
- Recreating the legacy custom alert event or Cap'n Proto enum.
- Enabling the feature by default.
- Reverse engineering a different `PRE_COLLISION_2` layout for newer Toyota architectures.
- Changing normal brake-hold behavior for other vehicle brands.

## User experience

### Local vehicle settings

Add an `Automatic Brake Hold` toggle to the existing Toyota/Lexus vehicle settings page. The parameter key is `ToyotaAutoHold`, and its default value is false.

Enabling the toggle requires confirmation because the feature commands braking. Changing the toggle requests a new onroad cycle so the CAN parsers, controller, and safety configuration are created from one consistent parameter value.

The toggle is enabled only when all of the following are true:

- `ui_state.CP` is available;
- the platform has `ToyotaFlags.TSS2`;
- the platform does not have `ToyotaFlags.RADAR_ACC`;
- the platform does not have `ToyotaFlags.SECOC`;
- `CP.openpilotLongitudinalControl` is true;
- `ToyotaEnforceStockLongitudinal` is false;
- the vehicle is not engaged.

When compatibility cannot be determined because the vehicle has not been started, the toggle is disabled and displays the existing start-the-vehicle compatibility message. When the platform is incompatible or factory longitudinal control is selected, the toggle is disabled and its stored parameter is cleared.

### Sunnylink settings

Add the same `ToyotaAutoHold` setting to the Toyota section of the Sunnylink settings schema. It requires an onroad cycle and uses capability and parameter gates matching the local UI. Regenerate `settings_ui.json` from the YAML source; do not edit the generated JSON manually.

### Driver alert

Do not add a custom event. Continue using the existing `CarState.brakeHoldActive` feedback from `ESP_CONTROL` and the existing openpilot `brakeHold` event. This reports confirmed vehicle state rather than merely reporting that the controller requested brake hold.

## Eligibility and feature configuration

Add `AUTO_BRAKE_HOLD` to the existing `ToyotaFlagsSP` enum using the next free bit. Add `AUTO_BRAKE_HOLD` to `ToyotaSafetyFlagsSP` using the next free safety bit.

Add `ToyotaAutoHold` to the parameter initialization list so its value is available before `CarInterface` creates CAN parsers and the controller.

During Toyota interface initialization, set both flags only when:

```python
auto_brake_hold_enabled = (
  params["ToyotaAutoHold"] and
  CP.flags & ToyotaFlags.TSS2 and
  not CP.flags & (ToyotaFlags.RADAR_ACC | ToyotaFlags.SECOC) and
  CP.openpilotLongitudinalControl
)
```

Parameter decoding must follow the repository's existing string-to-boolean convention. If factory longitudinal control disables openpilot longitudinal control, auto brake hold is not enabled and the safety flag is not set.

The rest of the implementation consumes the resolved `CP_SP.flags` capability. It does not read `Params` directly from opendbc controller code.

## State machine

### Inputs

The controller uses:

- `CS.out.standstill`;
- `CS.out.brakePressed`;
- `CS.out.gasPressed`;
- `CS.out.cruiseState.available`;
- `CS.out.cruiseState.enabled`;
- `CS.out.gearShifter`.

### Arming condition

The feature may accumulate activation time only while all of these are true:

```python
arming_allowed = (
  CS.out.standstill and
  CS.out.brakePressed and
  CS.out.cruiseState.available and
  not CS.out.gasPressed and
  not CS.out.cruiseState.enabled and
  CS.out.gearShifter not in (GearShifter.park, GearShifter.reverse)
)
```

The timer does not accumulate while the vehicle is moving, even if the brake is held. Once `standstill` becomes true, the brake must remain pressed continuously for 500 ms.

Use `DT_CTRL` rather than a magic frame count:

```python
AUTO_BRAKE_HOLD_ACTIVATION_FRAMES = round(0.5 / DT_CTRL)
```

Activation occurs when the counter reaches this value. If the brake is released or any other arming condition becomes false before activation, reset the counter to zero.

### Latched active state

Once active, brake-pedal state no longer controls the latch:

- releasing the brake keeps auto hold active;
- pressing the brake again keeps auto hold active;
- repeatedly pressing and releasing the brake keeps auto hold active.

This deliberately removes the legacy rising-edge cancellation behavior and its `_brake_hold_reset` state.

### Release condition

Release immediately when any of these is true:

- `not CS.out.standstill`;
- `CS.out.gasPressed`;
- `CS.out.cruiseState.enabled`;
- `not CS.out.cruiseState.available`;
- gear is park or reverse.

On release, clear both the active latch and activation counter. Neutral, sport, low, and brake gear retain the legacy behavior of being allowed unless represented as park or reverse by the platform.

### Transition summary

```text
INACTIVE
  ├─ arming conditions false ───────────────► INACTIVE, counter = 0
  └─ standstill + brake held ───────────────► ARMING

ARMING
  ├─ any arming condition false before 0.5s ► INACTIVE, counter = 0
  └─ continuous brake hold reaches 0.5s ────► ACTIVE

ACTIVE
  ├─ brake released or pressed again ───────► ACTIVE
  └─ gas/movement/ACC/main-off/P/R ─────────► INACTIVE, counter = 0
```

## CAN input and message preservation

### Source message

`PRE_COLLISION_2` is address `0x344`, length 8, received from camera bus 2 at approximately 33 Hz.

The current DBC already defines this message but only contains the signals used by `create_pcs_commands`. Extend `_toyota_adas_standard.dbc` with the legacy signals required for lossless signal-level reconstruction:

- `DS1STAT2`
- `DS1STBK2`
- `PCSWAR`
- `PCSOPR`
- `PCSABK`
- `PPTRGR`
- `CLEXTRGR`
- `IRLT_REQ`
- `BRKHLD`
- `VGRSTRGR`
- `PBRTRGR`
- `PCSDIS`
- `PBPREPMP`

The existing `DSS1GDRV`, `PCSALM`, `IBTRGR`, `PBATRGR`, `PREFILL`, `AVSTRGR`, and `CHECKSUM` definitions remain unchanged.

Run the repository's DBC generator after updating the source and verify all Toyota generated DBCs that import `_toyota_adas_standard.dbc` remain valid.

### Parser and first-frame guard

When the resolved feature flag is enabled, add `PRE_COLLISION_2` at 33 Hz to the camera parser.

Only update the saved signal dictionary when `vl_all` shows that a valid `PRE_COLLISION_2` frame arrived during the current parser update. Track an explicit `pre_collision_2_seen` boolean.

The controller must not send a replacement `0x344` before the first valid camera frame. This prevents an all-zero synthetic message from replacing stock PCS data during startup.

Save the parser timestamp of the most recent valid frame. Treat the source as stale after 100 ms, measured against the `now_nanos` value supplied to `CarController.update`. When stale, stop controller transmission so the panda forwarding watchdog can restore direct stock forwarding.

### Output message

Send the replacement `PRE_COLLISION_2` on bus 0 every second 100 Hz controller frame, for a nominal rate of 50 Hz.

When auto hold is inactive, copy every defined non-checksum signal from the most recent valid camera message. Let `CANPacker` recalculate the Toyota checksum.

When active, preserve the legacy actuation values:

```python
values = {
  "DSS1GDRV": 0x3FF,
  "PBRTRGR": frame % 730 < 727,
}
```

All unspecified active-message signals remain zero, matching the legacy implementation. The unusual `DSS1GDRV` literal and the 730-frame pulse are intentionally preserved rather than reinterpreted without vehicle data.

## Controller structure

Put the custom behavior in a focused sunnypilot Toyota module, following the existing gas-interceptor composition pattern. The module owns:

- activation timing and latch state;
- release logic;
- inactive signal preservation;
- active signal generation;
- the 50 Hz send schedule;
- the first-camera-frame guard.

The stock Toyota `CarController` initializes and calls this component but does not contain the state-machine implementation itself. This keeps the custom feature isolated from upstream Toyota control logic and permits direct unit testing.

The existing Toyota `CarStateExt` owns the saved camera message and seen flag because they are sunnypilot-specific state.

## Safety and forwarding

### Existing safety behavior

The current Toyota longitudinal safety configuration already permits `0x344`, bus 0, length 8. Stock-longitudinal and SecOC configurations do not expose this feature because eligibility rejects them.

No global `ALLOW_AEB` alternative-experience flag is added. The feature is represented by the Toyota-specific safety flag.

### Duplicate-message prevention

Without additional forwarding logic, stock camera `0x344` and the controller-generated `0x344` would both reach bus 0. Add a Toyota forwarding hook that blocks camera-to-powertrain `0x344` only when:

- the auto-brake-hold safety flag is enabled; and
- a valid controller `0x344` transmission has been observed recently.

The controller transmits every 20 ms. Use a conservative 100 ms forwarding watchdog. If no controller `0x344` is observed for more than 100 ms, resume forwarding the stock camera message.

This gives the following behavior:

- before the controller has its first valid camera sample: stock forwarding continues;
- while the controller is healthy: only the reconstructed controller message reaches bus 0;
- after controller failure or message timeout: stock forwarding resumes automatically;
- while the feature is disabled: forwarding remains unchanged.

Record the last accepted bus-0 `0x344` timestamp in the Toyota TX hook. The forwarding hook uses wrap-safe elapsed-time helpers already used by the safety framework.

The safety forwarding watchdog does not attempt to interpret AEB strength. The reconstructed inactive frame preserves all known stock signals, while watchdog fallback restores direct forwarding if the host stops transmitting.

## Error handling and failure behavior

- Missing first camera frame: do not transmit a replacement; leave stock forwarding active.
- Camera parser timeout: after 100 ms without a valid source frame, stop controller transmission and allow the safety forwarding watchdog to restore stock forwarding.
- Controller/process failure: the 100 ms safety watchdog restores stock `0x344` forwarding.
- Incompatible platform or factory longitudinal selection: do not set controller or safety flags; clear/disable the UI parameter.
- Parameter changed: request a new onroad cycle rather than hot-swapping parser and safety state.
- DBC checksum: always generated by `CANPacker`; never copy the parsed checksum value.

## Testing strategy

### State-machine unit tests

Test the focused controller component directly:

- no activation while moving with brake pressed;
- activation at exactly 500 ms of continuous brake after standstill;
- releasing before 500 ms resets the timer;
- releasing after activation keeps the latch active;
- pressing the brake again after activation keeps the latch active;
- gas releases immediately;
- movement releases immediately;
- ACC activation releases immediately;
- cruise-main off releases immediately;
- park and reverse release immediately;
- first valid cycle after release starts a new 500 ms timer.

### CAN tests

- no replacement is sent before the first valid camera message;
- inactive message preserves every source signal and has a valid checksum;
- active message contains the legacy `DSS1GDRV` and `PBRTRGR` values;
- output address, bus, size, and 50 Hz schedule are correct;
- 730-frame `PBRTRGR` behavior matches the legacy implementation;
- stale or invalid camera data stops host replacement transmission.

### Eligibility tests

- camera-ACC TSS2 plus enabled parameter sets controller and safety flags;
- radar-ACC TSS2 is rejected;
- SecOC TSS2 is rejected;
- factory longitudinal mode is rejected;
- false parameter is rejected.

### Safety tests

- with feature flag off, camera `0x344` forwards normally;
- with feature flag on but no recent host message, camera `0x344` forwards normally;
- after a valid host `0x344`, camera `0x344` is blocked;
- before 100 ms the message remains blocked;
- after 100 ms it forwards again;
- invalid bus/address/length do not refresh the watchdog;
- stock-longitudinal safety still rejects host `0x344`.

### UI and settings tests

- `ToyotaAutoHold` exists in the params registry and defaults false;
- local Toyota settings include the toggle;
- the toggle is disabled for radar-ACC, SecOC, factory longitudinal, unavailable `CP`, and engaged state;
- confirmation writes the parameter and requests an onroad cycle;
- Sunnylink source schema contains the setting and generated JSON matches it.

### Regression suites

Run at minimum:

- Toyota car tests;
- Toyota safety tests;
- DBC generator consistency tests;
- params tests;
- local UI tests covering Toyota settings;
- Sunnylink settings schema and change tests;
- ruff on all modified Python files.

## Baseline observations

Before implementation, the Toyota safety suite completed successfully with 1,118 tests and 134 skips. The Sunnylink settings tests could not start because the macOS `libparams_c.dylib` had not yet been built; this is an environment/build prerequisite rather than a test assertion failure and must be resolved before final verification.
