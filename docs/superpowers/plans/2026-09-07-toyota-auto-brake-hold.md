# Toyota Auto Brake Hold Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Add UI-controlled Toyota Auto Brake Hold to current sunnypilot `dev`, activating after the driver holds the brake for 500 ms at standstill and remaining latched across later brake presses.

**Architecture:** Resolve eligibility once into Toyota-specific controller and safety flags, parse and preserve camera `PRE_COLLISION_2`, and delegate the state machine/message schedule to a focused sunnypilot Toyota controller component. Add a 100 ms panda forwarding watchdog so controller traffic replaces stock `0x344` only while the host is healthy, otherwise stock forwarding fails open.

**Tech Stack:** Python 3.12, unittest, opendbc CANParser/CANPacker, Toyota DBC generator, panda C safety hooks, Raylib Python UI, Sunnylink YAML schema compiler.

**Spec:** `docs/superpowers/specs/2026-09-07-toyota-auto-brake-hold-design.md`

## Global Constraints

- Base all work on commit `e45bdf10aca0a14b01cf912d2387fffd9f897522` from `origin/dev`.
- Keep `ToyotaAutoHold` disabled by default.
- Support only TSS2/LSS2 platforms without `RADAR_ACC` or `SECOC`, and only while openpilot longitudinal control is configured.
- Start the 500 ms timer only after `standstill` and `brakePressed` are both true.
- Once active, brake release and later brake presses must not clear the latch.
- Gas, movement, ACC enable, cruise-main off, park, and reverse must release immediately.
- Do not add a custom Cap'n Proto event; use existing `brakeHoldActive` feedback.
- Do not use global `ALLOW_AEB` to enable the feature.
- Preserve the legacy active CAN values `DSS1GDRV=0x3FF` and `PBRTRGR=frame % 730 < 727`.
- Do not send replacement `0x344` before a valid camera frame or after the source has been stale for 100 ms.
- Keep feature-specific production code and enum members neutrally named; existing framework types such as `CarParamsSP` and `ToyotaFlagsSP` remain unchanged.

---

### Task 1: Resolve Toyota feature eligibility before interface construction

**Files:**
- Create: `opendbc_repo/opendbc/sunnypilot/car/toyota/tests/__init__.py`
- Create: `opendbc_repo/opendbc/sunnypilot/car/toyota/tests/test_auto_brake_hold.py`
- Create: `opendbc_repo/opendbc/sunnypilot/car/toyota/auto_brake_hold.py`
- Modify: `opendbc_repo/opendbc/sunnypilot/car/toyota/values.py`
- Modify: `opendbc_repo/opendbc/sunnypilot/car/interfaces.py`
- Modify: `openpilot/sunnypilot/selfdrive/car/interfaces.py`

**Interfaces:**
- Produces: `is_auto_brake_hold_available(CP: structs.CarParams) -> bool`.
- Produces: `ToyotaFlagsSP.AUTO_BRAKE_HOLD = 32`.
- Produces: `ToyotaSafetyFlagsSP.AUTO_BRAKE_HOLD = 4`.
- Produces: resolved controller and safety flags before `CarInterface` creates parsers.

- [ ] **Step 1: Write failing eligibility tests**

Add table-driven tests using real `structs.CarParams` and `structs.CarParamsSP`. The test helper should construct Toyota params with `openpilotLongitudinalControl=True`, then call `_initialize_toyota` with string parameter values.

```python
class TestAutoBrakeHoldEligibility(unittest.TestCase):
  def make_cp(self, flags=ToyotaFlags.TSS2):
    cp = structs.CarParams()
    cp.brand = "toyota"
    cp.flags = int(flags)
    cp.openpilotLongitudinalControl = True
    cp.safetyConfigs = [structs.CarParams.SafetyConfig()]
    return cp

  def test_camera_acc_tss2_is_enabled(self):
    cp, cp_sp = self.make_cp(), structs.CarParamsSP()
    _initialize_toyota(cp, cp_sp, {"ToyotaAutoHold": "1"})
    self.assertTrue(cp_sp.flags & ToyotaFlagsSP.AUTO_BRAKE_HOLD)
    self.assertTrue(cp_sp.safetyParam & ToyotaSafetyFlagsSP.AUTO_BRAKE_HOLD)

  def test_incompatible_configurations_are_rejected(self):
    cases = (
      (ToyotaFlags.TSS2 | ToyotaFlags.RADAR_ACC, True),
      (ToyotaFlags.TSS2 | ToyotaFlags.SECOC, True),
      (ToyotaFlags.TSS2, False),
    )
    for flags, openpilot_long in cases:
      cp, cp_sp = self.make_cp(flags), structs.CarParamsSP()
      cp.openpilotLongitudinalControl = openpilot_long
      _initialize_toyota(cp, cp_sp, {"ToyotaAutoHold": "1"})
      self.assertFalse(cp_sp.flags & ToyotaFlagsSP.AUTO_BRAKE_HOLD)
      self.assertFalse(cp_sp.safetyParam & ToyotaSafetyFlagsSP.AUTO_BRAKE_HOLD)

  def test_disabled_parameter_is_rejected(self):
    cp, cp_sp = self.make_cp(), structs.CarParamsSP()
    _initialize_toyota(cp, cp_sp, {"ToyotaAutoHold": "0"})
    self.assertFalse(cp_sp.flags & ToyotaFlagsSP.AUTO_BRAKE_HOLD)
```

- [ ] **Step 2: Run the new tests and verify RED**

Run:

```bash
uv run python opendbc_repo/opendbc/sunnypilot/car/toyota/tests/test_auto_brake_hold.py
```

Expected: import or attribute failure because the module and enum members do not exist.

- [ ] **Step 3: Implement the availability predicate and flags**

In `auto_brake_hold.py`:

```python
from opendbc.car import structs
from opendbc.car.toyota.values import ToyotaFlags


def is_auto_brake_hold_available(CP: structs.CarParams) -> bool:
  incompatible = ToyotaFlags.RADAR_ACC | ToyotaFlags.SECOC
  return bool(
      CP.brand == "toyota" and
      CP.flags & ToyotaFlags.TSS2 and
      not CP.flags & incompatible and
      CP.openpilotLongitudinalControl
  )
```

Add the neutral enum members to the existing framework enum classes:

```python
class ToyotaFlagsSP(IntFlag):
  SMART_DSU = 1
  RADAR_CAN_FILTER = 2
  ZSS = 4
  STOCK_LONGITUDINAL = 8
  STOP_AND_GO_HACK = 16
  AUTO_BRAKE_HOLD = 32


class ToyotaSafetyFlagsSP:
  DEFAULT = 0
  UNSUPPORTED_DSU = 1
  GAS_INTERCEPTOR = 2
  AUTO_BRAKE_HOLD = 4
```

Add `ToyotaAutoHold` to the Toyota block returned by `initialize_params`. Extend `_initialize_toyota`:

```python
toyota_auto_hold = int(params_dict.get("ToyotaAutoHold", 0)) == 1
if toyota_auto_hold and is_auto_brake_hold_available(CP):
  CP_SP.flags |= ToyotaFlagsSP.AUTO_BRAKE_HOLD.value
  CP_SP.safetyParam |= ToyotaSafetyFlagsSP.AUTO_BRAKE_HOLD
```

- [ ] **Step 4: Run eligibility tests and verify GREEN**

Run the Task 1 unittest command and verify all cases pass.

- [ ] **Step 5: Run focused lint**

```bash
uv run ruff check opendbc_repo/opendbc/sunnypilot/car/toyota/auto_brake_hold.py \
  opendbc_repo/opendbc/sunnypilot/car/toyota/tests/test_auto_brake_hold.py \
  opendbc_repo/opendbc/sunnypilot/car/interfaces.py
```

- [ ] **Step 6: Commit Task 1**

```bash
git add opendbc_repo/opendbc/sunnypilot/car/toyota/tests \
  opendbc_repo/opendbc/sunnypilot/car/toyota/auto_brake_hold.py \
  opendbc_repo/opendbc/sunnypilot/car/toyota/values.py \
  opendbc_repo/opendbc/sunnypilot/car/interfaces.py \
  openpilot/sunnypilot/selfdrive/car/interfaces.py
git commit -m "feat(toyota): configure automatic brake hold"
```

---

### Task 2: Complete `PRE_COLLISION_2` DBC coverage and message packing

**Files:**
- Modify: `opendbc_repo/opendbc/dbc/generator/toyota/_toyota_adas_standard.dbc`
- Modify: generated Toyota `*_generated.dbc` files produced by the generator
- Modify: `opendbc_repo/opendbc/sunnypilot/car/toyota/auto_brake_hold.py`
- Modify: `opendbc_repo/opendbc/sunnypilot/car/toyota/tests/test_auto_brake_hold.py`

**Interfaces:**
- Produces: `PRE_COLLISION_2_SIGNALS: tuple[str, ...]` containing every non-checksum legacy signal.
- Produces: `create_auto_brake_hold_command(packer, frame, stock_values, active) -> CanData`.

- [ ] **Step 1: Write failing DBC and packing tests**

Use a real `CANPacker` and `CANParser`. Assert the DBC exposes all 19 non-checksum source signals and that inactive output parses back to hand-specified values. Separately assert active output decodes with `PBRTRGR=1` on frame 0 and `PBRTRGR=0` on frame 728.

```python
EXPECTED_SIGNALS = {
  "DSS1GDRV", "DS1STAT2", "DS1STBK2", "PCSWAR", "PCSALM",
  "PCSOPR", "PCSABK", "PBATRGR", "PPTRGR", "IBTRGR",
  "CLEXTRGR", "IRLT_REQ", "BRKHLD", "AVSTRGR", "VGRSTRGR",
  "PREFILL", "PBRTRGR", "PCSDIS", "PBPREPMP",
}

def test_pre_collision_2_has_all_passthrough_signals(self):
  packer = CANPacker("toyota_nodsu_pt_generated")
  self.assertEqual(EXPECTED_SIGNALS | {"CHECKSUM"}, set(packer.dbc.name_to_msg["PRE_COLLISION_2"].sigs))

def test_inactive_command_preserves_stock_signals(self):
  stock = {
    "DSS1GDRV": -0.5, "DS1STAT2": 3, "DS1STBK2": 2,
    "PCSWAR": 1, "PCSALM": 1, "PCSOPR": 1, "PCSABK": 1,
    "PBATRGR": 2, "PPTRGR": 1, "IBTRGR": 1, "CLEXTRGR": 1,
    "IRLT_REQ": 2, "BRKHLD": 1, "AVSTRGR": 1, "VGRSTRGR": 2,
    "PREFILL": 1, "PBRTRGR": 1, "PCSDIS": 1, "PBPREPMP": 1,
  }
  msg = create_auto_brake_hold_command(self.packer, 0, stock, False)
  parser = CANParser("toyota_nodsu_pt_generated", [("PRE_COLLISION_2", 33)], 0)
  parser.update([1, [msg]])
  for signal, expected in stock.items():
    self.assertEqual(expected, parser.vl["PRE_COLLISION_2"][signal])
```

Do not compute expected values using `PRE_COLLISION_2_SIGNALS`; the literal fixture above must remain independent of the production signal list.

- [ ] **Step 2: Run the packing tests and verify RED**

Expected: missing DBC signals and missing helper.

- [ ] **Step 3: Add the missing DBC signals**

Add these exact definitions to `_toyota_adas_standard.dbc`, retaining the existing signals and `CHECKSUM`:

```dbc
 SG_ DS1STAT2 : 13|3@0+ (1,0) [0|0] "" Vector__XXX
 SG_ DS1STBK2 : 10|3@0+ (1,0) [0|0] "" Vector__XXX
 SG_ PCSWAR : 18|1@0+ (1,0) [0|0] "" FCM
 SG_ PCSOPR : 16|1@0+ (1,0) [0|0] "" Vector__XXX
 SG_ PCSABK : 31|1@0+ (1,0) [0|0] "" Vector__XXX
 SG_ PPTRGR : 28|1@0+ (1,0) [0|0] "" FCM
 SG_ CLEXTRGR : 26|1@0+ (1,0) [0|0] "" Vector__XXX
 SG_ IRLT_REQ : 25|2@0+ (1,0) [0|0] "" Vector__XXX
 SG_ BRKHLD : 37|1@0+ (1,0) [0|0] "" Vector__XXX
 SG_ VGRSTRGR : 35|2@0+ (1,0) [0|0] "" Vector__XXX
 SG_ PBRTRGR : 32|1@0+ (1,0) [0|0] "" Vector__XXX
 SG_ PCSDIS : 43|1@0+ (1,0) [0|0] "" Vector__XXX
 SG_ PBPREPMP : 40|1@0+ (1,0) [0|0] "" Vector__XXX
```

- [ ] **Step 4: Regenerate all DBC outputs**

```bash
uv run python opendbc_repo/opendbc/dbc/generator/generator.py
```

Review the generated diff and verify changes are limited to Toyota DBCs importing `_toyota_adas_standard.dbc` plus deterministic generator output.

- [ ] **Step 5: Implement the packer helper**

```python
PRE_COLLISION_2_SIGNALS = (
  "DSS1GDRV", "DS1STAT2", "DS1STBK2", "PCSWAR", "PCSALM",
  "PCSOPR", "PCSABK", "PBATRGR", "PPTRGR", "IBTRGR",
  "CLEXTRGR", "IRLT_REQ", "BRKHLD", "AVSTRGR", "VGRSTRGR",
  "PREFILL", "PBRTRGR", "PCSDIS", "PBPREPMP",
)


def create_auto_brake_hold_command(packer, frame: int, stock_values: dict[str, float], active: bool):
  values = {signal: stock_values[signal] for signal in PRE_COLLISION_2_SIGNALS}
  if active:
    values = {
      "DSS1GDRV": 0x3FF,
      "PBRTRGR": frame % 730 < 727,
    }
  return packer.make_can_msg("PRE_COLLISION_2", 0, values)
```

- [ ] **Step 6: Run packing tests and DBC consistency tests**

```bash
uv run python opendbc_repo/opendbc/sunnypilot/car/toyota/tests/test_auto_brake_hold.py
uv run python -m unittest discover opendbc_repo/opendbc/can/tests
```

- [ ] **Step 7: Commit Task 2**

Stage the DBC source, generated outputs, helper, and tests. Commit:

```bash
git commit -m "feat(toyota): pack brake hold PCS messages"
```

---

### Task 3: Implement the 500 ms state machine and camera-source guard

**Files:**
- Modify: `opendbc_repo/opendbc/sunnypilot/car/toyota/auto_brake_hold.py`
- Modify: `opendbc_repo/opendbc/sunnypilot/car/toyota/carstate_ext.py`
- Modify: `opendbc_repo/opendbc/car/toyota/carstate.py`
- Modify: `opendbc_repo/opendbc/car/toyota/carcontroller.py`
- Modify: `opendbc_repo/opendbc/sunnypilot/car/toyota/tests/test_auto_brake_hold.py`

**Interfaces:**
- Produces: `AutoBrakeHoldController(CP, CP_SP)`.
- Consumes: `CS.pre_collision_2`, `CS.pre_collision_2_seen`, `CS.pre_collision_2_ts_nanos`.
- Produces: `update(CS, packer, frame, now_nanos) -> list[CanData]`.

- [ ] **Step 1: Write failing state-machine tests**

Create minimal dataclasses representing the complete state consumed by the controller. Use literal sequences and the real controller object. Cover these independent mutations:

```python
@dataclass
class CruiseState:
  available: bool = True
  enabled: bool = False


@dataclass
class Out:
  standstill: bool = True
  brakePressed: bool = False
  gasPressed: bool = False
  gearShifter: GearShifter = GearShifter.drive
  cruiseState: CruiseState = field(default_factory=CruiseState)


@dataclass
class State:
  out: Out = field(default_factory=Out)
  pre_collision_2: dict[str, float] = field(default_factory=dict)
  pre_collision_2_seen: bool = False
  pre_collision_2_ts_nanos: int = 0


def activate(self):
  self.state.out.standstill = True
  self.state.out.brakePressed = True
  for _ in range(round(0.5 / DT_CTRL)):
    self.controller.update_state(self.state)
  self.assertTrue(self.controller.active)


def test_timer_starts_only_after_standstill(self):
  self.state.out.brakePressed = True
  self.state.out.standstill = False
  for frame in range(100):
    self.controller.update_state(self.state)
  self.assertFalse(self.controller.active)
  self.assertEqual(0, self.controller.activation_frames)

def test_activates_after_500_ms_at_standstill(self):
  self.state.out.brakePressed = True
  self.state.out.standstill = True
  for _ in range(round(0.5 / DT_CTRL) - 1):
    self.controller.update_state(self.state)
  self.assertFalse(self.controller.active)
  self.controller.update_state(self.state)
  self.assertTrue(self.controller.active)

def test_brake_release_before_activation_resets_timer(self):
  self.state.out.brakePressed = True
  self.state.out.standstill = True
  for _ in range(25):
    self.controller.update_state(self.state)
  self.state.out.brakePressed = False
  self.controller.update_state(self.state)
  self.assertEqual(0, self.controller.activation_frames)
  self.assertFalse(self.controller.active)

def test_brake_release_after_activation_keeps_active(self):
  self.activate()
  self.state.out.brakePressed = False
  self.controller.update_state(self.state)
  self.assertTrue(self.controller.active)

def test_second_brake_press_keeps_active(self):
  self.activate()
  self.state.out.brakePressed = False
  self.controller.update_state(self.state)
  self.state.out.brakePressed = True
  self.controller.update_state(self.state)
  self.assertTrue(self.controller.active)
```

Add separate table-driven release tests for gas, movement, ACC enable, cruise-main off, park, and reverse. Each test must first activate the real state machine, mutate one input, then assert immediate release and counter reset.

- [ ] **Step 2: Write failing source-freshness and schedule tests**

With a real Toyota packer, assert:

- no messages when `pre_collision_2_seen=False`;
- no messages when `now_nanos - pre_collision_2_ts_nanos > 100_000_000`;
- one message on even frames with fresh source data;
- no message on odd frames.

- [ ] **Step 3: Run Task 3 tests and verify RED**

Expected: missing controller methods and missing CarState integration.

- [ ] **Step 4: Implement the focused controller**

Use these constants and transition order:

```python
ACTIVATION_FRAMES = round(0.5 / DT_CTRL)
SOURCE_TIMEOUT_NANOS = 100_000_000

def update_state(self, CS):
  disallowed_gear = CS.out.gearShifter in (GearShifter.park, GearShifter.reverse)
  release = (not CS.out.standstill or CS.out.gasPressed or CS.out.cruiseState.enabled or
             not CS.out.cruiseState.available or disallowed_gear)
  if release:
    self.activation_frames = 0
    self.active = False
  elif not self.active:
    if CS.out.brakePressed:
      self.activation_frames += 1
      self.active = self.activation_frames >= ACTIVATION_FRAMES
    else:
      self.activation_frames = 0
```

After updating state, send only when enabled, the source was seen, source age is between zero and 100 ms inclusive, and `frame % 2 == 0`.

- [ ] **Step 5: Integrate CarState source capture**

Initialize `pre_collision_2`, `pre_collision_2_seen`, and `pre_collision_2_ts_nanos` in `CarStateExt`. In its update method, only copy data when:

```python
len(cp_cam.vl_all["PRE_COLLISION_2"]["DSS1GDRV"]) > 0
```

Set the timestamp from `cp_cam.ts_nanos["PRE_COLLISION_2"]["DSS1GDRV"]`. Add `("PRE_COLLISION_2", 33)` to camera messages only for the resolved feature flag.

- [ ] **Step 6: Integrate the controller component**

Compose `AutoBrakeHoldController` alongside `GasInterceptorCarController`, initialize it with `CP`/`CP_SP`, and extend `can_sends` from its `update` method on every controller cycle. Pass the existing `now_nanos` argument unchanged.

- [ ] **Step 7: Run state, Toyota interface, and lint tests**

```bash
uv run python opendbc_repo/opendbc/sunnypilot/car/toyota/tests/test_auto_brake_hold.py
uv run python opendbc_repo/opendbc/car/toyota/tests/test_toyota.py
uv run ruff check opendbc_repo/opendbc/car/toyota/carcontroller.py \
  opendbc_repo/opendbc/car/toyota/carstate.py \
  opendbc_repo/opendbc/sunnypilot/car/toyota/auto_brake_hold.py \
  opendbc_repo/opendbc/sunnypilot/car/toyota/carstate_ext.py \
  opendbc_repo/opendbc/sunnypilot/car/toyota/tests/test_auto_brake_hold.py
```

- [ ] **Step 8: Commit Task 3**

```bash
git commit -m "feat(toyota): activate brake hold after stopped braking"
```

---

### Task 4: Add fail-open panda forwarding arbitration

**Files:**
- Modify: `opendbc_repo/opendbc/safety/modes/toyota.h`
- Modify: `opendbc_repo/opendbc/safety/tests/test_toyota.py`

**Interfaces:**
- Consumes: `ToyotaSafetyFlagsSP.AUTO_BRAKE_HOLD` through `current_safety_param_sp`.
- Produces: camera bus-2 `0x344` forwarding blocked only while accepted host bus-0 `0x344` is newer than or equal to the 100 ms timeout.

- [ ] **Step 1: Write failing forwarding watchdog tests**

Add a dedicated safety class that enables `ToyotaSafetyFlagsSP.AUTO_BRAKE_HOLD` before `set_safety_hooks`. Use `set_timer` and real packed `PRE_COLLISION_2` frames:

```python
class TestToyotaAutoBrakeHoldSafety(TestToyotaSafetyTorque):
  SAFETY_PARAM_SP = ToyotaSafetyFlagsSP.AUTO_BRAKE_HOLD

  def test_brake_hold_forwarding_watchdog(self):
    self.safety.set_timer(1)
    self.assertEqual(0, self.safety.safety_fwd_hook(2, 0x344))

    msg = self.packer.make_can_msg_safety("PRE_COLLISION_2", 0, {"DSS1GDRV": 0x3FF, "PBRTRGR": 1})
    self.assertTrue(self._tx(msg))
    self.assertEqual(-1, self.safety.safety_fwd_hook(2, 0x344))

    self.safety.set_timer(100_001)
    self.assertEqual(-1, self.safety.safety_fwd_hook(2, 0x344))
    self.safety.set_timer(100_002)
    self.assertEqual(0, self.safety.safety_fwd_hook(2, 0x344))
```

Add tests proving feature-off forwarding is unchanged and wrong bus/address/length cannot refresh the timestamp.

- [ ] **Step 2: Run the focused safety test and verify RED**

```bash
uv run python opendbc_repo/opendbc/safety/tests/test_toyota.py TestToyotaAutoBrakeHoldSafety
```

Expected: camera `0x344` remains forwarded immediately after host transmission.

- [ ] **Step 3: Implement safety state and watchdog**

Add module state:

```c
static bool toyota_auto_brake_hold = false;
static bool toyota_brake_hold_tx_seen = false;
static uint32_t toyota_brake_hold_tx_ts = 0U;
const uint32_t TOYOTA_BRAKE_HOLD_TX_TIMEOUT = 100000U;
```

In `toyota_init`, resolve safety flag `4`, reset the seen flag/timestamp, and disable the feature if stock longitudinal or SecOC is active.

At the end of `toyota_tx_hook`, update the timestamp only when `tx` remains true and the message is bus 0/address `0x344`/length 8.

Add and register a forwarding hook:

```c
static bool toyota_fwd_hook(int bus_num, int addr) {
  bool block = false;
  if (toyota_auto_brake_hold && toyota_brake_hold_tx_seen && (bus_num == 2) && (addr == 0x344)) {
    uint32_t elapsed = safety_get_ts_elapsed(microsecond_timer_get(), toyota_brake_hold_tx_ts);
    block = elapsed <= TOYOTA_BRAKE_HOLD_TX_TIMEOUT;
  }
  return block;
}
```

Use the exact return type required by the current `safety_hooks.fwd` declaration; if it is `bool`, retain the code above.

- [ ] **Step 4: Run focused and full Toyota safety suites**

```bash
uv run python opendbc_repo/opendbc/safety/tests/test_toyota.py TestToyotaAutoBrakeHoldSafety
uv run python opendbc_repo/opendbc/safety/tests/test_toyota.py
```

Expected full baseline: 1,118 existing tests plus the newly added cases, with no failures.

- [ ] **Step 5: Commit Task 4**

```bash
git add opendbc_repo/opendbc/safety/modes/toyota.h opendbc_repo/opendbc/safety/tests/test_toyota.py
git commit -m "safety(toyota): arbitrate brake hold PCS forwarding"
```

---

### Task 5: Add local and Sunnylink UI controls

**Files:**
- Modify: `openpilot/common/params_keys.h`
- Modify: `openpilot/selfdrive/ui/sunnypilot/layouts/settings/vehicle/brands/toyota.py`
- Modify: `openpilot/sunnypilot/sunnylink/capabilities.py`
- Modify: `openpilot/sunnypilot/sunnylink/settings_ui_src/pages/vehicle.yaml`
- Modify: `openpilot/sunnypilot/sunnylink/settings_ui.json` (generated)
- Modify: `openpilot/sunnypilot/sunnylink/tests/test_capabilities.py`
- Modify: `openpilot/sunnypilot/sunnylink/tests/test_settings_schema.py`
- Modify: `openpilot/sunnypilot/sunnylink/tests/test_settings_changes.py`

**Interfaces:**
- Produces: persistent boolean param `ToyotaAutoHold`, default false.
- Produces: local Toyota toggle with confirmation and onroad-cycle request.
- Produces: Sunnylink capability `toyota_auto_brake_hold_available` and gated remote toggle.

- [ ] **Step 1: Write failing param and capability tests**

Extend existing behavior tests to assert:

```python
self.assertFalse(Params().get_bool("ToyotaAutoHold"))
```

Add literal capability cases for compatible camera-ACC TSS2, radar-ACC, SecOC, and missing CP. Assert `toyota_auto_brake_hold_available` is true only for the compatible case.

In schema tests, require `ToyotaAutoHold` in Toyota keys and require its enablement rules to reference both `toyota_auto_brake_hold_available` and `ToyotaEnforceStockLongitudinal == false`.

- [ ] **Step 2: Run focused settings tests and verify RED**

Build the missing native params library first if necessary:

```bash
uv run scons -j4 openpilot/common
uv run python -m unittest \
  openpilot.sunnypilot.sunnylink.tests.test_capabilities \
  openpilot.sunnypilot.sunnylink.tests.test_settings_schema \
  openpilot.sunnypilot.sunnylink.tests.test_settings_changes
```

Expected: missing param, capability, and UI schema entries.

- [ ] **Step 3: Register the parameter**

Add:

```cpp
{"ToyotaAutoHold", {PERSISTENT | BACKUP, BOOL, "0"}},
```

next to the other Toyota parameters.

- [ ] **Step 4: Add the local Toyota toggle**

Add the description and control beside the existing Toyota controls:

```python
DESCRIPTIONS = {
  # existing descriptions
  'auto_brake_hold': tr_noop(
    'Hold the brake for 500 ms after the vehicle stops to activate brake hold. '
    'Accelerator input, movement, ACC, cruise-main off, park, or reverse releases it.'
  ),
}

self.auto_brake_hold = toggle_item_sp(
  lambda: tr("Automatic Brake Hold"),
  description=lambda: tr(DESCRIPTIONS["auto_brake_hold"]),
  initial_state=ui_state.params.get_bool("ToyotaAutoHold"),
  callback=self._on_enable_auto_brake_hold,
  enabled=lambda: not ui_state.engaged,
)
```

Implement the confirmation callback explicitly:

```python
def _on_enable_auto_brake_hold(self, state: bool):
  if state:
    def confirm_callback(result: int):
      enabled = result == DialogResult.CONFIRM
      ui_state.params.put_bool("ToyotaAutoHold", enabled)
      self.auto_brake_hold.action_item.set_state(enabled)
      if enabled:
        ui_state.params.put_bool("OnroadCycleRequested", True)

    content = (f"<h1>{self.auto_brake_hold.title}</h1><br>" +
               f"<p>{self.auto_brake_hold.description}</p>")
    gui_app.push_widget(ConfirmDialog(content, tr("Enable"), rich=True, callback=confirm_callback))
  else:
    ui_state.params.put_bool("ToyotaAutoHold", False)
    ui_state.params.put_bool("OnroadCycleRequested", True)
```

When factory longitudinal is enabled, also clear `ToyotaAutoHold` and update the auto-hold action state in `_on_enable_enforce_stock_longitudinal`.

In `update_settings`, call the shared `is_auto_brake_hold_available(ui_state.CP)` predicate. Disable and clear the setting if unavailable or if factory longitudinal is selected. Never permit changes while `ui_state.engaged`.

- [ ] **Step 5: Add Sunnylink capability and YAML item**

Add `toyota_auto_brake_hold_available` to `CAPABILITY_FIELDS` and `CAPABILITY_LABELS`. Resolve it from the selected Toyota platform flags or authoritative `CarParams`, using the same predicate semantics as the local UI.

Add the YAML toggle under Toyota:

```yaml
- key: ToyotaAutoHold
  widget: toggle
  needs_onroad_cycle: true
  title: Automatic Brake Hold
  description: Hold the brake for 500 ms after the vehicle stops to activate brake hold. Accelerator input, movement, ACC, cruise-main off, park, or reverse releases it.
  enablement:
  - type: capability
    field: toyota_auto_brake_hold_available
    equals: true
  - type: param
    key: ToyotaEnforceStockLongitudinal
    equals: false
  - $ref: '#/macros/not_engaged'
```

- [ ] **Step 6: Regenerate and validate Sunnylink JSON**

```bash
uv run python openpilot/sunnypilot/sunnylink/tools/compile_settings_ui.py
uv run python openpilot/sunnypilot/sunnylink/tools/validate_settings_ui.py
```

- [ ] **Step 7: Run settings tests and lint**

```bash
uv run python -m unittest \
  openpilot.sunnypilot.sunnylink.tests.test_capabilities \
  openpilot.sunnypilot.sunnylink.tests.test_compile_settings_ui \
  openpilot.sunnypilot.sunnylink.tests.test_settings_schema \
  openpilot.sunnypilot.sunnylink.tests.test_settings_changes
uv run ruff check openpilot/selfdrive/ui/sunnypilot/layouts/settings/vehicle/brands/toyota.py \
  openpilot/sunnypilot/sunnylink/capabilities.py \
  openpilot/sunnypilot/sunnylink/tests/test_capabilities.py \
  openpilot/sunnypilot/sunnylink/tests/test_settings_schema.py \
  openpilot/sunnypilot/sunnylink/tests/test_settings_changes.py
```

- [ ] **Step 8: Commit Task 5**

```bash
git commit -m "feat(ui): add Toyota automatic brake hold setting"
```

---

### Task 6: Final integration and regression verification

**Files:**
- Modify only if a verification failure identifies a defect in files already listed above.

**Interfaces:**
- Verifies the complete parameter → eligibility → parser → controller → CAN → panda forwarding flow.

- [ ] **Step 1: Run all focused feature tests**

```bash
uv run python opendbc_repo/opendbc/sunnypilot/car/toyota/tests/test_auto_brake_hold.py
uv run python opendbc_repo/opendbc/car/toyota/tests/test_toyota.py
uv run python opendbc_repo/opendbc/safety/tests/test_toyota.py
uv run python -m unittest \
  openpilot.sunnypilot.sunnylink.tests.test_capabilities \
  openpilot.sunnypilot.sunnylink.tests.test_compile_settings_ui \
  openpilot.sunnypilot.sunnylink.tests.test_settings_schema \
  openpilot.sunnypilot.sunnylink.tests.test_settings_changes
```

- [ ] **Step 2: Verify generated artifacts are current**

```bash
uv run python opendbc_repo/opendbc/dbc/generator/generator.py
uv run python openpilot/sunnypilot/sunnylink/tools/compile_settings_ui.py --check
git diff --exit-code
```

Run the generators before `git diff --exit-code`; the working tree must remain unchanged after generation.

- [ ] **Step 3: Run lint and format checks over every modified Python file**

```bash
git diff --name-only origin/dev...HEAD -- '*.py' | xargs uv run ruff check
git diff --check
```

- [ ] **Step 4: Review safety-critical invariants manually**

Confirm from the final diff:

- `ToyotaAutoHold` defaults false;
- only resolved compatible platforms set controller and safety flags;
- timer begins only after standstill plus brake;
- brake changes cannot clear an active latch;
- all five release categories remain present;
- stale source stops host `0x344` within 100 ms;
- safety resumes stock forwarding after a further 100 ms without host traffic;
- no custom event enum or global AEB alternative experience was added.

- [ ] **Step 5: Run branch diff review**

```bash
git status --short --branch
git diff --stat origin/dev...HEAD
git log --oneline origin/dev..HEAD
```

- [ ] **Step 6: Commit any verification fixes and rerun affected tests**

Do not commit generated drift or unrelated formatting. If no fix is required, leave the task complete without an empty commit.
