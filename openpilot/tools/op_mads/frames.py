#!/usr/bin/env python3
"""Front camera frames at route times, captioned with speed, commanded acceleration, lead and model plan.

usage: frames.py <route dir> <seconds> [<seconds> ...] [-o out.jpg]
Seconds count from the route start, as drive_report.py prints them. Missing qcamera.ts segments are copied from the
device (OP_MADS_DEVICE, default comma_4). Decoding needs PyAV (`uv pip install av` in the test copy venv).
"""
import argparse
import math
import os
import subprocess
from pathlib import Path

from PIL import Image, ImageDraw

from openpilot.tools.op_mads.drive_report import SERVICES, _Series, _at, _of, load_route, route_t0

DEVICE = os.environ.get("OP_MADS_DEVICE", "comma_4")
REALDATA = "/data/media/0/realdata"
ENCODE_IDX = "qNarrowRoadEncodeIdx"
HEADER = 44  # px of caption above each frame
COLS = 4


def frame_jobs(enc, times) -> list[tuple[float, int, int]]:
  """(time, segment, frame index in that segment's qcamera.ts) for each time with an encoded frame at or before it."""
  jobs = []
  for t in times:
    e = _at(enc, t)
    if e is not None:
      jobs.append((t, e.segmentNum, e.segmentId))
  return jobs


def captions(msgs, t: float, t0: float) -> tuple[str, str]:
  car, cc = _at(_of(msgs, "carState"), t), _at(_of(msgs, "carControl"), t)
  radar, model, ss = _at(_of(msgs, "radarState"), t), _at(_of(msgs, "modelV2"), t), _at(_of(msgs, "selfdriveState"), t)
  v = f"{car.vEgo * 3.6:.0f} km/h" if car is not None else "-- km/h"
  accel = f"accel {cc.actuators.accel:+.1f}" if cc is not None else "accel --"
  lead = f"lead {radar.leadOne.dRel:.1f}m" if radar is not None and radar.leadOne.present else "no lead"
  plan = f"plan {model.position.x[-1]:.0f}m" if model is not None and len(model.position.x) else "plan --"
  if ss is not None and ss.enabled:
    mode = "op long, experimental" if ss.experimentalMode else "op long"
  else:
    mode = "driver long"
  return f"{t - t0:.1f}s  {v}  {accel}", f"{lead}  {plan}  {mode}"


def contact_sheet(tiles, cols: int = COLS) -> Image.Image:
  """tiles: (image, caption lines); laid out in a grid, cols wide."""
  w, h = tiles[0][0].size
  rows = math.ceil(len(tiles) / cols)
  sheet = Image.new("RGB", (min(len(tiles), cols) * w, rows * (h + HEADER)), "black")
  draw = ImageDraw.Draw(sheet)
  for i, (img, lines) in enumerate(tiles):
    x, y = (i % cols) * w, (i // cols) * (h + HEADER)
    sheet.paste(img, (x, y + HEADER))
    for k, line in enumerate(lines):
      draw.text((x + 6, y + 4 + k * 18), line, fill="yellow" if k == 0 else "white")
  return sheet


def fetch_segment_file(route: Path, seg: int, name: str = "qcamera.ts") -> Path:
  """A file of the segment, copied from the device if it is not there yet."""
  dst = route / str(seg) / name
  if not dst.is_file():
    part = dst.with_name(name + ".part")
    subprocess.run(["scp", "-q", "-o", "ConnectTimeout=8", f"{DEVICE}:{REALDATA}/{route.name}--{seg}/{name}", str(part)], check=True)
    part.rename(dst)
  return dst


def decode(path: Path, indices: set[int]) -> dict[int, Image.Image]:
  import av
  frames = {}
  with av.open(str(path)) as container:
    for i, frame in enumerate(container.decode(video=0)):
      if i in indices:
        frames[i] = frame.to_image()
      if i >= max(indices):
        break
  return frames


def main() -> None:
  parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
  parser.add_argument("route_dir")
  parser.add_argument("seconds", nargs="+", type=float)
  parser.add_argument("-o", "--output", help="default: <route dir>/frames_<first second>.jpg")
  args = parser.parse_args()

  route = Path(args.route_dir)
  t0 = route_t0(str(route))
  msgs = load_route(str(route), SERVICES | {ENCODE_IDX})
  jobs = frame_jobs(_Series((t, e) for t, e in _of(msgs, ENCODE_IDX)), [t0 + s for s in args.seconds])
  images = {}
  for seg in sorted({seg for _, seg, _ in jobs}):
    for idx, img in decode(fetch_segment_file(route, seg), {i for _, s, i in jobs if s == seg}).items():
      images[(seg, idx)] = img
  tiles = [(images[(seg, idx)], list(captions(msgs, t, t0))) for t, seg, idx in jobs if (seg, idx) in images]
  out = Path(args.output or route / f"frames_{args.seconds[0]:.0f}.jpg")
  contact_sheet(tiles).save(out, quality=85)
  print(out)


if __name__ == "__main__":
  main()
