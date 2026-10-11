#pragma once

#include "opendbc/safety/declarations.h"

// MADS: lateral-only engagement, enabled by ALT_EXP_ENABLE_MADS.
// Only supported on the Toyota safety mode: the ACC main parse that gates lateral is Toyota only.
// Invariant: lateral permission implies ACC main is on. It is granted on a rising edge of ACC main or of
// controls_allowed (only while ACC main is on), and revoked whenever ACC main is off, on rx check failures,
// on safety mode changes, and when openpilot stops reporting MADS in its heartbeat. Brake and gas never affect it.
// Toyota SecOC does not parse ACC main (acc_main_on stays false), so it never grants lateral.
// Longitudinal control keeps using controls_allowed only.

bool controls_allowed_lateral = false;
bool heartbeat_engaged_mads = false;              // openpilot has MADS enabled (paused counts), heartbeat param2; must be 0 when selfdriveStateSP is stale
uint32_t heartbeat_engaged_mads_mismatches = 0U;  // count of mismatches between heartbeat_engaged_mads and controls_allowed_lateral

static bool mads_acc_main_prev = false;
static bool mads_controls_allowed_prev = false;

static void mads_reset(void) {
  controls_allowed_lateral = false;
  heartbeat_engaged_mads_mismatches = 0U;
  mads_acc_main_prev = false;
  mads_controls_allowed_prev = false;
}

// called once per rx message, after the safety mode rx hook updated acc_main_on and controls_allowed
static void mads_rx_update(void) {
  const bool mads_enabled = ((alternative_experience & ALT_EXP_ENABLE_MADS) != 0) && (current_safety_mode == SAFETY_TOYOTA);
  const bool acc_main_rising = acc_main_on && !mads_acc_main_prev;
  const bool controls_allowed_rising = controls_allowed && !mads_controls_allowed_prev;

  if (!mads_enabled || !acc_main_on) {
    controls_allowed_lateral = false;
  } else if (acc_main_rising || controls_allowed_rising) {
    controls_allowed_lateral = true;
    heartbeat_engaged_mads_mismatches = 0U;
  } else {
    // no edge: keep the current state
  }

  mads_acc_main_prev = acc_main_on;
  mads_controls_allowed_prev = controls_allowed;
}

// 1Hz: exit lateral control if openpilot has not reported MADS as enabled for a few seconds
void mads_heartbeat_engaged_check(void) {
  if (controls_allowed_lateral && !heartbeat_engaged_mads) {
    heartbeat_engaged_mads_mismatches += 1U;
    if (heartbeat_engaged_mads_mismatches >= 3U) {
      controls_allowed_lateral = false;
    }
  } else {
    heartbeat_engaged_mads_mismatches = 0U;
  }
}
