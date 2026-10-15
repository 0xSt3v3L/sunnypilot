import unittest
from unittest.mock import MagicMock, patch

import numpy as np

from openpilot.selfdrive.navigation.google import (FIELD_MASK, ROUTES_URL, compute_route, decode_polyline, maneuver,
                                                   route_request, steps_from_route)
from openpilot.selfdrive.navigation.helpers import valhalla_type


def encode_polyline(points) -> str:
  """(longitude, latitude) points as a Google encoded polyline, for building responses."""
  out, prev = [], (0, 0)
  for lon, lat in points:
    cur = (round(lat * 1e5), round(lon * 1e5))
    for value in (cur[0] - prev[0], cur[1] - prev[1]):
      value = ~(value << 1) if value < 0 else value << 1
      while value >= 0x20:
        out.append(chr((0x20 | (value & 0x1f)) + 63))
        value >>= 5
      out.append(chr(value + 63))
    prev = cur
  return ''.join(out)


def google_step(points, distance, seconds, google_maneuver=None, text=''):
  step = {'distanceMeters': distance, 'staticDuration': f'{seconds}s', 'polyline': {'encodedPolyline': encode_polyline(points)}}
  if google_maneuver is not None:
    step['navigationInstruction'] = {'maneuver': google_maneuver, 'instructions': text}
  return step


# A depart east, a left turn north, then a right turn east again; about 111 m per 0.001 degree.
ROUTE = {'routes': [{'legs': [{'steps': [
  google_step([(121.000, 25.0), (121.002, 25.0)], 200, 20, 'DEPART', '向東行駛'),
  google_step([(121.002, 25.0), (121.002, 25.001)], 110, 15, 'TURN_LEFT', '左轉'),
  google_step([(121.002, 25.001), (121.003, 25.001)], 100, 10, 'TURN_RIGHT', '右轉'),
]}]}]}


class TestPolyline(unittest.TestCase):
  def test_google_example(self):
    # The example in Google's encoded polyline documentation, as (latitude, longitude).
    points = decode_polyline('_p~iF~ps|U_ulLnnqC_mqNvxq`@')
    np.testing.assert_allclose(points, [(-120.2, 38.5), (-120.95, 40.7), (-126.453, 43.252)])

  def test_round_trip(self):
    points = [(121.5645, 25.034), (121.517, 25.0478), (121.51701, 25.04781)]
    np.testing.assert_allclose(decode_polyline(encode_polyline(points)), points, atol=1e-6)

  def test_empty(self):
    self.assertEqual(decode_polyline('').shape, (0, 2))


class TestManeuvers(unittest.TestCase):
  def test_valhalla_commands(self):
    # The Valhalla command the model sees for each Google maneuver, through the branch's own Mapbox conversion.
    expected = {
      'TURN_LEFT': 15, 'TURN_RIGHT': 10, 'TURN_SLIGHT_LEFT': 16, 'TURN_SLIGHT_RIGHT': 9,
      'TURN_SHARP_LEFT': 14, 'TURN_SHARP_RIGHT': 11, 'UTURN_LEFT': 13, 'UTURN_RIGHT': 13,
      'STRAIGHT': 8, 'NAME_CHANGE': 8, 'MANEUVER_UNSPECIFIED': 8,
      'FORK_LEFT': 24, 'FORK_RIGHT': 23, 'ROUNDABOUT_LEFT': 26, 'ROUNDABOUT_RIGHT': 26,
      'FERRY': 28, 'DEPART': 1, 'MERGE': None,
    }
    for google_maneuver, command in expected.items():
      with self.subTest(google_maneuver):
        self.assertEqual(valhalla_type(*maneuver(google_maneuver, previous_speed=10.)), command)

  def test_ramps(self):
    # Google has one ramp maneuver; leaving a fast road it is an exit, otherwise an on ramp.
    self.assertEqual(valhalla_type(*maneuver('RAMP_RIGHT', previous_speed=25.)), 20)
    self.assertEqual(valhalla_type(*maneuver('RAMP_LEFT', previous_speed=25.)), 21)
    self.assertEqual(valhalla_type(*maneuver('RAMP_RIGHT', previous_speed=12.)), 18)
    self.assertEqual(valhalla_type(*maneuver('RAMP_LEFT', previous_speed=12.)), 19)

  def test_unknown_is_straight(self):
    self.assertEqual(valhalla_type(*maneuver('SOMETHING_NEW', previous_speed=10.)), 8)


class TestSteps(unittest.TestCase):
  def test_mapbox_shaped_steps(self):
    steps = steps_from_route(ROUTE)
    self.assertEqual([(s['maneuver']['type'], s['maneuver'].get('modifier', '')) for s in steps],
                     [('depart', ''), ('turn', 'left'), ('turn', 'right'), ('arrive', '')])
    self.assertEqual([s['distance'] for s in steps], [200, 110, 100, 0])
    self.assertEqual([s['duration'] for s in steps], [20, 15, 10, 0])
    np.testing.assert_allclose(steps[1]['geometry']['coordinates'], [(121.002, 25.0), (121.002, 25.001)])
    # the arrival is a zero-length line at the destination, as Mapbox gives it: navd measures along it
    np.testing.assert_allclose(steps[3]['geometry']['coordinates'], [(121.003, 25.001), (121.003, 25.001)])

  def test_banners_announce_the_next_maneuver(self):
    # As in Mapbox, a step's banner describes the maneuver at its end, shown over the whole step.
    steps = steps_from_route(ROUTE)
    banners = [s['bannerInstructions'] for s in steps]
    self.assertEqual(banners[0], [{'distanceAlongGeometry': 200, 'primary': {'text': '左轉', 'type': 'turn', 'modifier': 'left'}}])
    self.assertEqual(banners[1][0]['primary'], {'text': '右轉', 'type': 'turn', 'modifier': 'right'})
    self.assertEqual(banners[2][0]['primary'], {'text': '', 'type': 'arrive', 'modifier': ''})
    self.assertEqual(banners[3], [])

  def test_ramp_uses_the_previous_step_speed(self):
    route = {'routes': [{'legs': [{'steps': [
      google_step([(121.0, 25.0), (121.01, 25.0)], 1000, 36, 'DEPART'),  # 100 km/h
      google_step([(121.01, 25.0), (121.011, 25.0)], 100, 10, 'RAMP_RIGHT'),
    ]}]}]}
    self.assertEqual(steps_from_route(route)[1]['maneuver'], {'type': 'off ramp', 'modifier': 'right'})

  def test_legs_are_joined(self):
    route = {'routes': [{'legs': [ROUTE['routes'][0]['legs'][0], {'steps': [
      google_step([(121.003, 25.001), (121.004, 25.001)], 100, 10, 'STRAIGHT')]}]}]}
    steps = steps_from_route(route)
    self.assertEqual(len(steps), 5)
    self.assertEqual(steps[-1]['maneuver']['type'], 'arrive')

  def test_no_route(self):
    self.assertEqual(steps_from_route({}), [])
    self.assertEqual(steps_from_route({'routes': []}), [])


class TestRequest(unittest.TestCase):
  def test_request_body(self):
    body = route_request((121.0, 25.0), (121.1, 25.1), heading=91.6, vias=[(121.05, 25.05)], language='zh-TW')
    self.assertEqual(body['origin'], {'location': {'latLng': {'latitude': 25.0, 'longitude': 121.0}, 'heading': 92}})
    self.assertEqual(body['destination'], {'location': {'latLng': {'latitude': 25.1, 'longitude': 121.1}}})
    self.assertEqual(body['intermediates'], [{'location': {'latLng': {'latitude': 25.05, 'longitude': 121.05}}, 'via': True}])
    self.assertEqual((body['travelMode'], body['languageCode'], body['units']), ('DRIVE', 'zh-TW', 'METRIC'))

  def test_via_heading(self):
    # a via with the direction of travel keeps Google from reaching it the other way round with a U-turn
    body = route_request((121.0, 25.0), (121.1, 25.1), vias=[(121.05, 25.05, 359.8)])
    self.assertEqual(body['intermediates'], [{'location': {'latLng': {'latitude': 25.05, 'longitude': 121.05}, 'heading': 0}, 'via': True}])

  def test_request_without_heading_or_vias(self):
    body = route_request((121.0, 25.0), (121.1, 25.1))
    self.assertNotIn('heading', body['origin']['location'])
    self.assertNotIn('intermediates', body)

  def test_heading_wraps(self):
    self.assertEqual(route_request((0, 0), (1, 1), heading=-0.2)['origin']['location']['heading'], 0)
    self.assertEqual(route_request((0, 0), (1, 1), heading=359.7)['origin']['location']['heading'], 0)

  def test_compute_route_posts_with_key_header(self):
    response = MagicMock()
    response.json.return_value = ROUTE
    with patch('openpilot.selfdrive.navigation.google.requests.post', return_value=response) as post:
      steps = compute_route((121.0, 25.0), (121.003, 25.001), api_key='test-key')
    self.assertEqual(len(steps), 4)
    url, kwargs = post.call_args.args[0], post.call_args.kwargs
    self.assertEqual(url, ROUTES_URL)
    self.assertEqual(kwargs['headers']['X-Goog-Api-Key'], 'test-key')
    self.assertEqual(kwargs['headers']['X-Goog-FieldMask'], FIELD_MASK)
    self.assertNotIn('test-key', url)  # never in a URL, which ends up in logs
    response.raise_for_status.assert_called_once()


if __name__ == '__main__':
  unittest.main()
