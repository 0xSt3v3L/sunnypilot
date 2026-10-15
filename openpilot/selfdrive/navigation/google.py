"""Google Routes API routes as the Mapbox-shaped steps navd and the navigation model input expect."""
import os
from pathlib import Path

import numpy as np
import requests

from openpilot.common.hardware import PC
from openpilot.common.hardware.hw import Paths

ROUTES_URL = 'https://routes.googleapis.com/directions/v2:computeRoutes'
KEY_PATH = Path(Paths.comma_home() if PC else '/data') / 'navigation' / 'google_maps_api_key'
FIELD_MASK = ','.join('routes.legs.steps.' + f for f in ('distanceMeters', 'staticDuration', 'polyline.encodedPolyline',
                                                         'navigationInstruction'))
RAMP_EXIT_SPEED = 70 / 3.6  # m/s; a ramp taken from a road at least this fast is an exit, otherwise an on ramp

# Google maneuver -> Mapbox (type, modifier); helpers.valhalla_type turns these into the model's Valhalla commands
MANEUVERS = {
  'TURN_LEFT': ('turn', 'left'), 'TURN_RIGHT': ('turn', 'right'),
  'TURN_SLIGHT_LEFT': ('turn', 'slight left'), 'TURN_SLIGHT_RIGHT': ('turn', 'slight right'),
  'TURN_SHARP_LEFT': ('turn', 'sharp left'), 'TURN_SHARP_RIGHT': ('turn', 'sharp right'),
  'UTURN_LEFT': ('turn', 'uturn'), 'UTURN_RIGHT': ('turn', 'uturn'),
  'STRAIGHT': ('continue', 'straight'), 'NAME_CHANGE': ('new name', 'straight'),
  'FORK_LEFT': ('fork', 'left'), 'FORK_RIGHT': ('fork', 'right'),
  'MERGE': ('merge', ''),
  'ROUNDABOUT_LEFT': ('roundabout', 'left'), 'ROUNDABOUT_RIGHT': ('roundabout', 'right'),
  'FERRY': ('notification', ''), 'FERRY_TRAIN': ('notification', ''),
  'DEPART': ('depart', ''),
}


def load_api_key(path: Path = KEY_PATH) -> str | None:
  """The Google Maps Platform key: GOOGLE_MAPS_API_KEY, else the key file put on the device by hand."""
  try:
    return os.environ.get('GOOGLE_MAPS_API_KEY') or path.read_text().strip() or None
  except OSError:
    return None


def decode_polyline(encoded: str) -> np.ndarray:
  """A Google encoded polyline as an (n, 2) array of (longitude, latitude)."""
  values, value, shift = [], 0, 0
  for char in encoded:
    b = ord(char) - 63
    value |= (b & 0x1f) << shift
    shift += 5
    if b < 0x20:
      values.append(~(value >> 1) if value & 1 else value >> 1)
      value, shift = 0, 0
  lat_lon = np.cumsum(np.array(values, dtype=np.float64).reshape((-1, 2)), axis=0) / 1e5
  return lat_lon[:, ::-1].reshape((-1, 2))


def maneuver(google_maneuver: str, previous_speed: float) -> tuple[str, str]:
  """The Mapbox (type, modifier) of a Google maneuver; previous_speed (m/s) tells an exit ramp from an on ramp."""
  if google_maneuver in ('RAMP_LEFT', 'RAMP_RIGHT'):
    side = 'left' if google_maneuver == 'RAMP_LEFT' else 'right'
    return ('off ramp' if previous_speed >= RAMP_EXIT_SPEED else 'on ramp'), side
  return MANEUVERS.get(google_maneuver, ('continue', 'straight'))


def steps_from_route(response: dict) -> list[dict]:
  """The first route's steps with Mapbox fields, ending in an arrive step at the destination; [] without a route."""
  routes = response.get('routes') or []
  if not routes:
    return []
  steps, previous_speed = [], 0.
  for google_step in (s for leg in routes[0].get('legs', []) for s in leg.get('steps', [])):
    instruction = google_step.get('navigationInstruction', {})
    kind, modifier = ('depart', '') if not steps else maneuver(instruction.get('maneuver', ''), previous_speed)
    distance = google_step.get('distanceMeters', 0)
    duration = float(google_step.get('staticDuration', '0s').rstrip('s'))
    steps.append({
      'distance': distance,
      'duration': duration,
      'geometry': {'coordinates': decode_polyline(google_step['polyline']['encodedPolyline']).tolist()},
      'maneuver': {'type': kind, 'modifier': modifier},
      'text': instruction.get('instructions', ''),
    })
    previous_speed = distance / duration if duration > 0 else 0.
  if not steps:
    return []
  # the arrival is a zero-length line at the destination, as Mapbox gives it: navd measures along it
  steps.append({'distance': 0, 'duration': 0, 'geometry': {'coordinates': steps[-1]['geometry']['coordinates'][-1:] * 2},
                'maneuver': {'type': 'arrive', 'modifier': ''}, 'text': ''})
  # As in Mapbox, a step's banner announces the maneuver at its end; the arrive step has none
  for step, following in zip(steps[:-1], steps[1:], strict=True):
    step['bannerInstructions'] = [{'distanceAlongGeometry': step['distance'],
                                   'primary': {'text': following['text'], **following['maneuver']}}]
  steps[-1]['bannerInstructions'] = []
  for step in steps:
    del step['text']
  return steps


def _location(point, heading: float | None = None) -> dict:
  location = {'latLng': {'latitude': point[1], 'longitude': point[0]}}
  if heading is not None:
    location['heading'] = round(heading) % 360  # the direction of travel there, degrees clockwise from north
  return location


def route_request(origin, destination, heading: float | None = None, vias=(), language: str = 'zh-TW') -> dict:
  """A computeRoutes body from (longitude, latitude) points; vias, optionally (longitude, latitude, heading), steer
  the route without stopping."""
  body = {
    'origin': {'location': _location(origin, heading)},
    'destination': {'location': _location(destination)},
    'travelMode': 'DRIVE',
    'routingPreference': 'TRAFFIC_UNAWARE',
    'languageCode': language,
    'units': 'METRIC',
  }
  if len(vias):
    body['intermediates'] = [{'location': _location(p, p[2] if len(p) > 2 else None), 'via': True} for p in vias]
  return body


def compute_route(origin, destination, api_key: str, heading: float | None = None, vias=(), language: str = 'zh-TW',
                  timeout: float = 10) -> list[dict]:
  headers = {'X-Goog-Api-Key': api_key, 'X-Goog-FieldMask': FIELD_MASK}
  response = requests.post(ROUTES_URL, json=route_request(origin, destination, heading, vias, language), headers=headers,
                           timeout=timeout)
  response.raise_for_status()
  return steps_from_route(response.json())
