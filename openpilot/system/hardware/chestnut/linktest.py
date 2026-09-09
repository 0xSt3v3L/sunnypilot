#!/usr/bin/env python3
"""Chestnut GPU USB3 link-quality test.

Measures the accumulation rate of USB3 physical-layer link errors
(xHCI PORTLI "Link Error Count", the same value surfaced as
UsbState.linkErrorCount) on the chestnut accelerator's link. This is a hardware
counter: a healthy USB3 link stays ~flat (near 0/s), while a marginal
cable/connector/routing accumulates errors continuously, regardless of compute
load. Use it to compare wiring setups (as-routed vs re-routed vs a new cable)
by a number instead of by feel.

Kept dependency-free (stdlib + sysfs only, no openpilot imports) so it runs with
plain `python3` on the device in any state, without the full runtime env. The
sysfs reads mirror openpilot.common.hardware.usb.

Run onroad (ignition on) so the chestnut is powered and enumerated. Keep the
operating condition identical across the setups you compare so the rates are
comparable.

Usage:
  linktest.py [seconds] [label]
    seconds  measurement window (default 180)
    label    tag for the run, e.g. current / cable-out / new-cable
"""
import argparse
import glob
import os
import time

# idVendor (hex) of the chestnut running firmware; mirrors CHESTNUT_USB_IDS in
# openpilot.common.hardware.usb.
CHESTNUT_VENDORS = ("add1", "3801")
USB_DEVICES_PATH = "/sys/bus/usb/devices"
INTERVAL = 2.0
USB3_SPEED_MBPS = 5000
HEALTHY_RATE = 0.02  # errors/sec below which the link is effectively flat


def _read(path: str) -> str | None:
  try:
    with open(path) as f:
      return f.read().strip()
  except OSError:
    return None


def _read_int(path: str, base: int = 10) -> int | None:
  value = _read(path)
  try:
    return int(value, base)
  except (TypeError, ValueError):
    return None


def sample() -> dict | None:
  """Current chestnut link state, or None if it is not enumerated."""
  for device in glob.glob(os.path.join(USB_DEVICES_PATH, "*")):
    if _read(os.path.join(device, "idVendor")) not in CHESTNUT_VENDORS:
      continue
    controller = os.path.realpath(device)
    while controller != "/" and not os.path.basename(controller).endswith(".ssusb"):
      controller = os.path.dirname(controller)
    if not controller.endswith(".ssusb"):
      return None
    link_errors = _read_int(os.path.join(controller, "portli"), 0)
    return {
      "linkErrorCount": (link_errors & 0xFFFF) if link_errors is not None else None,
      "speedMbps": _read_int(os.path.join(device, "speed")),
    }
  return None


def main() -> int:
  parser = argparse.ArgumentParser(description="Measure the chestnut USB3 link-error accumulation rate")
  parser.add_argument("seconds", type=int, nargs="?", default=180, help="measurement window in seconds (default 180)")
  parser.add_argument("label", nargs="?", default="run", help="label for this run (e.g. current, cable-out, new-cable)")
  args = parser.parse_args()

  first = sample()
  if first is None or first["linkErrorCount"] is None:
    print("chestnut NOT enumerated. Turn ignition ON, wait ~30-60s for the big model, then re-run.")
    return 1

  start = time.monotonic()
  le_prev = first["linkErrorCount"]
  cum = 0        # link errors accrued during the window (survives counter resets)
  dropouts = 0   # samples where the chestnut vanished (link/enumeration lost)
  reenum = 0     # counter reset => device re-enumerated (link fully dropped)
  speeds: dict[int | None, int] = {}
  print(f"[{args.label}] start linkErrorCount={le_prev} speed={first['speedMbps']}Mbps  "
        f"measuring {args.seconds}s (Ctrl-C = stop early)", flush=True)

  try:
    while time.monotonic() - start < args.seconds:
      time.sleep(INTERVAL)
      current = sample()
      elapsed = time.monotonic() - start
      if current is None or current["linkErrorCount"] is None:
        dropouts += 1
        print(f"[{args.label}] t+{elapsed:5.0f}s  *** chestnut DROPPED OUT (link/enumeration lost) ***", flush=True)
        continue
      le, speed = current["linkErrorCount"], current["speedMbps"]
      speeds[speed] = speeds.get(speed, 0) + 1
      delta = le - le_prev
      if delta > 0:
        cum += delta
        print(f"[{args.label}] t+{elapsed:5.0f}s  linkErrorCount={le} (+{delta})  speed={speed}", flush=True)
      elif le < le_prev - 5:
        reenum += 1
        print(f"[{args.label}] t+{elapsed:5.0f}s  *** counter RESET {le_prev}->{le} (re-enumerated = link dropped) ***", flush=True)
      le_prev = le
  except KeyboardInterrupt:
    pass

  elapsed = time.monotonic() - start
  rate = cum / elapsed if elapsed > 0 else 0.0
  downgraded = any(speed and speed < USB3_SPEED_MBPS for speed in speeds)
  healthy = rate < HEALTHY_RATE and dropouts == 0 and reenum == 0 and not downgraded
  print("=" * 64, flush=True)
  print(f"[{args.label}] RESULT over {elapsed:.0f}s", flush=True)
  print(f"  link errors accrued: +{cum}", flush=True)
  print(f"  RATE = {rate:.3f} errors/sec  ({rate * 60:.1f}/min)", flush=True)
  print(f"  dropouts={dropouts}  re-enumerations={reenum}  speeds={speeds}", flush=True)
  print(f"  VERDICT: {'HEALTHY' if healthy else 'UNSTABLE'}  "
        f"(healthy: rate~0, no dropouts, speed stays {USB3_SPEED_MBPS})", flush=True)
  print("=" * 64, flush=True)
  return 0


if __name__ == "__main__":
  raise SystemExit(main())
