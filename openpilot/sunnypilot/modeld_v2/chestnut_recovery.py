"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""

# Pure decision logic for chestnut (big model) auto-recovery.
# Kept dependency-free so the safety-critical rules can be unit tested without hardware.

MAX_RECOVERY_ATTEMPTS = 2

RECOVERY_WAIT = "wait"
RECOVERY_RESTART = "restart"
RECOVERY_GIVE_UP = "give_up"


def is_controlling(carcontrol_alive: bool, lat_active: bool, long_active: bool) -> bool:
  """Whether openpilot is (or might be) actively controlling the car.

  Safety: if carControl is not alive we cannot confirm disengagement, so we assume it is controlling
  and refuse to restart modeld (which would blank the model output).
  """
  if not carcontrol_alive:
    return True
  return lat_active or long_active


def decide_recovery(recovery_pending: bool, engaged: bool, attempts: int,
                    max_attempts: int = MAX_RECOVERY_ATTEMPTS) -> str:
  """Decide what to do after the big model has fallen back to the small model.

  - RECOVERY_WAIT: do nothing this frame (nothing to recover, or openpilot is engaged).
  - RECOVERY_RESTART: safe to restart modeld to reload the big model.
  - RECOVERY_GIVE_UP: attempt budget for this ignition cycle exhausted; stay on the small model.

  Safety: never restart while engaged, so we never blank the model output while openpilot is controlling.
  """
  if not recovery_pending:
    return RECOVERY_WAIT
  if engaged:
    return RECOVERY_WAIT
  if attempts >= max_attempts:
    return RECOVERY_GIVE_UP
  return RECOVERY_RESTART
