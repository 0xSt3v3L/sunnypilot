from PIL import Image
from openpilot.cereal import log
from openpilot.common.test import OpenpilotTestCase
from openpilot.tools.op_mads.drive_report import _Series
from openpilot.tools.op_mads.frames import frame_jobs, captions, contact_sheet, HEADER


def at(t, **kwargs):
  msg = log.Event.new_message(**kwargs)
  msg.logMonoTime = int(round(t * 1e9))
  return msg.as_reader()


class TestFrames(OpenpilotTestCase):
  def test_frame_jobs_use_the_encode_index(self):
    enc = _Series((t, at(t, qNarrowRoadEncodeIdx={"segmentNum": int(t // 60), "segmentId": int(t % 60 * 20)}).qNarrowRoadEncodeIdx)
                  for t in [i * 0.05 for i in range(2400)])
    assert frame_jobs(enc, [10.02, 61.5]) == [(10.02, 0, 200), (61.5, 1, 30)]
    assert frame_jobs(enc, [-1.]) == []

  def test_captions(self):
    msgs = [at(0., carState={"vEgo": 10.}), at(0., radarState={"leadOne": {"present": True, "dRel": 12.3}}),
            at(0., modelV2={"position": {"x": [0., 20., 45.]}}), at(0., carControl={"actuators": {"accel": -1.24}}),
            at(0., selfdriveState={"enabled": True, "deprecated": {"experimentalMode": True}})]
    top, bottom = captions(msgs, 1.0, t0=0.)
    assert top == "1.0s  36 km/h  accel -1.2"
    assert bottom == "lead 12.3m  plan 45m  op long, experimental"

  def test_contact_sheet_grid(self):
    tiles = [(Image.new("RGB", (100, 50)), ["a", "b"]) for _ in range(5)]
    sheet = contact_sheet(tiles, cols=4)
    assert sheet.size == (400, 2 * (50 + HEADER))
