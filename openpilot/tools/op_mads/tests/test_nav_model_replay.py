import os
import tempfile
import unittest
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import patch

import numpy as np

from openpilot.selfdrive.navigation.helpers import VALHALLA_COMMAND_COUNT
from openpilot.selfdrive.navigation.renderer import SIZE
from openpilot.selfdrive.navigation.route import METERS_PER_DEGREE
from openpilot.tools.op_mads.nav_model_replay import TURN_VIA_OFFSET, NavInputs, ablation_inputs, api_key, flip_navigation, gps_fixes, \
  route_offsets, via_points

DEG = 1 / METERS_PER_DEGREE  # degrees of latitude per meter


def fix(lon, lat, speed=10., bearing=0., has_fix=True):
  return SimpleNamespace(longitude=lon, latitude=lat, speed=speed, bearingDeg=bearing, hasFix=has_fix)


def step(points, kind, modifier='', text=''):
  points = np.asarray(points, dtype=np.float64)
  return {'geometry': {'coordinates': points.tolist()}, 'maneuver': {'type': kind, 'modifier': modifier},
          'bannerInstructions': [{'distanceAlongGeometry': 0, 'primary': {'text': text, 'type': kind, 'modifier': modifier}}]}


# 200 m north, a right turn, then 100 m east; at the equator so degrees scale the same both ways
STEPS = [
  step([(0, 0), (0, 200 * DEG)], 'depart'),
  step([(0, 200 * DEG), (100 * DEG, 200 * DEG)], 'turn', 'right', 'turn right'),
  step([(100 * DEG, 200 * DEG)], 'arrive'),
]
STEPS[0]['bannerInstructions'][0]['primary']['text'] = 'turn right'


class FakeRenderer:
  def __init__(self, valid=True):
    self.valid, self.poses = valid, []

  def render(self, lon, lat, bearing, route):
    self.poses.append((lon, lat, bearing))
    return np.full((SIZE, SIZE), 7, dtype=np.uint8), self.valid


class TestGpsFixes(unittest.TestCase):
  def test_prefers_the_source_with_more_fixes(self):
    of = {'gpsLocation': [(1., fix(1, 1))],
          'gpsLocationExternal': [(1., fix(2, 2, 5., 90.)), (2., fix(3, 3, has_fix=False)), (3., fix(4, 4, 6., 91.))]}
    np.testing.assert_allclose(gps_fixes(of), [(1., 2, 2, 5., 90.), (3., 4, 4, 6., 91.)])

  def test_missing_services(self):
    self.assertEqual(gps_fixes({'gpsLocation': [(1., fix(1, 1))]}).shape, (1, 5))
    self.assertEqual(gps_fixes({}).shape, (0, 5))


class TestViaPoints(unittest.TestCase):
  def test_a_via_after_each_turn(self):
    # 300 m north, then 300 m east: one via past the corner, headed east
    track = np.array([(0, 0), (0, 300 * DEG), (300 * DEG, 300 * DEG)])
    vias = via_points(track)
    self.assertEqual(len(vias), 1)
    self.assertAlmostEqual(vias[0, 0] / DEG, TURN_VIA_OFFSET, delta=15)
    self.assertAlmostEqual(vias[0, 1] / DEG, 300, delta=1)
    self.assertAlmostEqual(vias[0, 2], 90, delta=1)
    self.assertAlmostEqual(vias[0, 3], 300 + TURN_VIA_OFFSET, delta=15)  # meters along the track

  def test_long_straights_are_filled(self):
    track = np.array([(0, 0), (0, 1000 * DEG)])
    vias = via_points(track)
    np.testing.assert_allclose(vias[:, 1] / DEG, [1000 / 3, 2000 / 3], atol=1e-6)
    np.testing.assert_allclose(vias[:, 2], [0, 0], atol=1e-6)

  def test_a_loop_keeps_each_pass(self):
    # around a 200 m block twice: a via after every corner
    square = [(0, 0), (0, 200), (200, 200), (200, 0), (0, 0)]
    track = np.array(square + square[1:]) * DEG
    self.assertEqual(len(via_points(track)), 7)

  def test_too_many_vias(self):
    zigzag = np.array([(100 * (i // 2 + i % 2), 100 * (i // 2)) for i in range(80)]) * DEG
    with self.assertRaises(ValueError):
      via_points(zigzag)

  def test_none_for_a_short_track(self):
    self.assertEqual(len(via_points(np.array([(0, 0)]))), 0)
    self.assertEqual(len(via_points(np.array([(0, 0), (0, 0)]))), 0)
    self.assertEqual(len(via_points(np.array([(0, 0), (0, 300 * DEG)]))), 0)


def static(steps):
  return lambda i: steps


class TestNavInputs(unittest.TestCase):
  def test_maneuver_ahead_and_map(self):
    renderer = FakeRenderer()
    nav = NavInputs(static(STEPS), np.array([(0., 0, 50 * DEG, 10., 0.)]), renderer)
    inputs = nav.at(0.5)  # half a second after the fix: 5 m further along
    self.assertEqual(inputs['navigation'].shape, (VALHALLA_COMMAND_COUNT + 1,))
    self.assertEqual(inputs['navigation'][10], 1)  # a right turn
    self.assertAlmostEqual(inputs['navigation'][-1], (200 - 50 - 5) / 100, places=3)
    self.assertEqual(inputs['nav_map'].dtype, np.uint8)
    self.assertTrue((inputs['nav_map'] == 7).all())
    lon, lat, bearing = renderer.poses[-1]
    self.assertAlmostEqual(lat / DEG, 55, places=3)
    self.assertAlmostEqual(bearing, 0, places=3)
    self.assertAlmostEqual(nav.progress, 50, places=3)

  def test_no_maneuver_after_the_last_turn(self):
    nav = NavInputs(static(STEPS), np.array([(0., 50 * DEG, 200 * DEG, 10., 90.)]), FakeRenderer())
    self.assertFalse(nav.at(0.)['navigation'].any())

  def test_invalid_map_is_black(self):
    nav = NavInputs(static(STEPS), np.array([(0., 0, 50 * DEG, 10., 0.)]), FakeRenderer(valid=False))
    self.assertFalse(nav.at(0.)['nav_map'].any())

  def test_no_inputs_without_a_recent_fix(self):
    nav = NavInputs(static(STEPS), np.array([(10., 0, 50 * DEG, 10., 0.)]), FakeRenderer())
    self.assertEqual(nav.at(9.), {})  # before the first fix
    self.assertEqual(nav.at(12.5), {})  # the fix is too old

  def test_maneuver_events(self):
    fixes = np.array([(float(t), 0, t * 20 * DEG, 20., 0.) for t in range(12)])
    nav = NavInputs(static(STEPS), fixes, FakeRenderer())
    for t in np.arange(0, 12, 0.5):
      nav.at(t)
    self.assertEqual(nav.events, [(10., 10, 'right', 'turn right')])  # 200 m at 20 m/s
    self.assertEqual(len(nav.maps), 1)  # the turn; the car never reaches the arrival
    self.assertEqual(nav.reroutes, 0)

  def test_progress_starts_at_the_route_start(self):
    # a route around a block and back past its start 2 m to the east: the first fix, nearer the later pass, is at the
    # route's start
    loop = [step([(0, 0), (0, 200 * DEG), (100 * DEG, 200 * DEG), (100 * DEG, 0), (2 * DEG, 0), (2 * DEG, 50 * DEG)], 'depart'),
            step([(2 * DEG, 50 * DEG)], 'arrive')]
    nav = NavInputs(static(loop), np.array([(0., 1.8 * DEG, 1 * DEG, 10., 0.)]), FakeRenderer())
    nav.at(0.)
    self.assertAlmostEqual(nav.progress, 1, places=3)

  def test_reroute_off_the_route(self):
    # the car heads east at the start while the first route goes north; navd reroutes after 3 s off it
    fixes = np.array([(float(t), t * 20 * DEG, 0, 20., 90.) for t in range(10)])
    east = [step([(0, 0), (300 * DEG, 0)], 'depart'), step([(300 * DEG, 0)], 'arrive')]
    calls = []

    def router(i):
      calls.append(i)
      return STEPS if len(calls) == 1 else east

    nav = NavInputs(router, fixes, FakeRenderer())
    for t in np.arange(0, 10, 0.5):
      nav.at(t)
    self.assertEqual(nav.reroutes, 1)
    self.assertEqual(calls[0], 0)
    self.assertGreaterEqual(calls[1], 4)  # the fix index at the reroute
    self.assertEqual(nav.events, [])


class TestAblation(unittest.TestCase):
  def test_flip_swaps_sides(self):
    for command, flipped in [(10, 15), (15, 10), (9, 16), (11, 14), (18, 19), (21, 20), (23, 24), (13, 13), (8, 8)]:
      with self.subTest(command=command):
        navigation = np.zeros(VALHALLA_COMMAND_COUNT + 1, dtype=np.float32)
        navigation[command], navigation[-1] = 1, 1.5
        out = flip_navigation(navigation)
        self.assertEqual(out[flipped], 1)
        self.assertEqual(out[:-1].sum(), 1)
        self.assertEqual(out[-1], 1.5)
        self.assertEqual(navigation[command], 1)  # the input is not changed

  def test_runs_share_one_update_per_frame(self):
    renderer = FakeRenderer()
    nav = NavInputs(static(STEPS), np.array([(0., 0, 50 * DEG, 10., 0.)]), renderer)
    runs = ablation_inputs(nav)
    self.assertEqual(list(runs), ['map only', 'instruction only', 'nav on', 'nav flipped'])
    out = {name: f(0.5) for name, f in runs.items()}
    self.assertEqual(len(renderer.poses), 1)
    self.assertEqual(set(out['map only']), {'nav_map'})
    self.assertEqual(set(out['instruction only']), {'navigation'})
    self.assertEqual(out['nav on']['navigation'][10], 1)
    self.assertEqual(out['nav flipped']['navigation'][15], 1)
    self.assertEqual(out['nav flipped']['navigation'][10], 0)
    self.assertIs(out['nav flipped']['nav_map'], out['nav on']['nav_map'])
    self.assertEqual({name: f(9.) for name, f in ablation_inputs(NavInputs(static(STEPS), np.array([(10., 0, 0, 1., 0.)]),
                                                                        FakeRenderer())).items()},
                     {'map only': {}, 'instruction only': {}, 'nav on': {}, 'nav flipped': {}})


class TestRouteOffsets(unittest.TestCase):
  def test_distance_from_the_route(self):
    route = np.array([(0, 0), (0, 200 * DEG)])
    track = np.array([(0, 10 * DEG), (5 * DEG, 100 * DEG)])
    np.testing.assert_allclose(route_offsets(track, route), [0, 5], atol=1e-3)


class TestApiKey(unittest.TestCase):
  def test_env_then_file(self):
    with tempfile.TemporaryDirectory() as d:
      path = Path(d) / 'key'
      path.write_text('from-file\n')
      with patch.dict(os.environ, {'GOOGLE_MAPS_API_KEY': 'from-env'}):
        self.assertEqual(api_key(path), 'from-env')
      with patch.dict(os.environ, {}, clear=True):
        self.assertEqual(api_key(path), 'from-file')
        with self.assertRaises(SystemExit):
          api_key(Path(d) / 'missing')


if __name__ == '__main__':
  unittest.main()
