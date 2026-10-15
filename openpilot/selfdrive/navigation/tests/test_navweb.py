import json
import tempfile
import threading
import unittest
from pathlib import Path
from unittest.mock import MagicMock, patch

import requests

from openpilot.selfdrive.navigation.navweb import (PLACES_URL, is_maps_url, load_token, make_server, parse_coordinates, resolve_link,
                                                   search_places, web_url)


class FakeParams:
  def __init__(self):
    self.values = {}

  def get(self, key):
    return self.values.get(key)

  def put(self, key, value):
    self.values[key] = value

  def remove(self, key):
    self.values.pop(key, None)


class TestParsing(unittest.TestCase):
  def test_coordinates(self):
    self.assertEqual(parse_coordinates('25.0478, 121.5170'), (25.0478, 121.517))
    self.assertEqual(parse_coordinates('25.0478,121.517'), (25.0478, 121.517))
    # a place link's pin (!3d/!4d) wins over the map view (@)
    url = 'https://www.google.com/maps/place/Taipei/@25.04,121.51,17z/data=!3m1!4b1!4m6!3m5!1s0x0:0x1!8m2!3d25.0478!4d121.517'
    self.assertEqual(parse_coordinates(url), (25.0478, 121.517))
    self.assertEqual(parse_coordinates('https://www.google.com/maps/@25.04,121.51,17z'), (25.04, 121.51))
    self.assertEqual(parse_coordinates('https://maps.google.com/?q=25.1,121.2'), (25.1, 121.2))
    self.assertEqual(parse_coordinates('https://www.google.com/maps/search/?api=1&query=25.1%2C121.2'), (25.1, 121.2))
    for text in ('Taipei 101', '', '95.0, 121.0', '25.0, 190.0', 'https://www.google.com/maps/place/Taipei+101'):
      with self.subTest(text):
        self.assertIsNone(parse_coordinates(text))

  def test_maps_urls(self):
    for url in ('https://maps.app.goo.gl/abc', 'https://goo.gl/maps/abc', 'https://www.google.com/maps/place/x', 'https://maps.google.com/?q=1,2',
                'https://google.com.tw/maps/place/x'):
      with self.subTest(url):
        self.assertTrue(is_maps_url(url))
    for url in ('https://evil.com/maps', 'https://google.com.evil.com/maps', 'http://169.254.169.254/', 'file:///etc/passwd', 'Taipei 101'):
      with self.subTest(url):
        self.assertFalse(is_maps_url(url))

  def test_resolve_short_link(self):
    final = 'https://www.google.com/maps/place/Taipei+101/@25.03,121.56,17z/data=!3d25.0339!4d121.5645'
    with patch('openpilot.selfdrive.navigation.navweb.requests.get', return_value=MagicMock(url=final)) as get:
      self.assertEqual(resolve_link('https://maps.app.goo.gl/abc'), ((25.0339, 121.5645), 'Taipei 101'))
    self.assertTrue(get.call_args.kwargs['allow_redirects'])
    with patch('openpilot.selfdrive.navigation.navweb.requests.get') as get:
      self.assertEqual(resolve_link('25.1, 121.2'), ((25.1, 121.2), ''))  # coordinates need no request
      self.assertEqual(resolve_link('https://evil.com/x'), (None, ''))
      get.assert_not_called()

  def test_resolve_follows_only_google(self):
    # a redirect off Google is not trusted
    with patch('openpilot.selfdrive.navigation.navweb.requests.get', return_value=MagicMock(url='https://evil.com/@25.1,121.2')):
      self.assertEqual(resolve_link('https://maps.app.goo.gl/abc'), (None, ''))


class TestSearch(unittest.TestCase):
  def test_search_places(self):
    response = MagicMock()
    response.json.return_value = {'places': [
      {'displayName': {'text': '台北101'}, 'formattedAddress': '台北市信義區', 'location': {'latitude': 25.0339, 'longitude': 121.5645}},
      {'displayName': {'text': 'no location'}},
    ]}
    with patch('openpilot.selfdrive.navigation.navweb.requests.post', return_value=response) as post:
      places = search_places('101', 'test-key', near=(25.0, 121.5))
    self.assertEqual(places, [{'name': '台北101', 'address': '台北市信義區', 'latitude': 25.0339, 'longitude': 121.5645}])
    self.assertEqual(post.call_args.args[0], PLACES_URL)
    self.assertEqual(post.call_args.kwargs['headers']['X-Goog-Api-Key'], 'test-key')
    body = post.call_args.kwargs['json']
    self.assertEqual((body['textQuery'], body['languageCode'], body['regionCode']), ('101', 'zh-TW', 'TW'))
    self.assertEqual(body['locationBias']['circle']['center'], {'latitude': 25.0, 'longitude': 121.5})
    response.raise_for_status.assert_called_once()
    with patch('openpilot.selfdrive.navigation.navweb.requests.post', return_value=response) as post:
      search_places('101', 'test-key')
    self.assertNotIn('locationBias', post.call_args.kwargs['json'])


class TestToken(unittest.TestCase):
  def test_created_once(self):
    with tempfile.TemporaryDirectory() as d:
      path = Path(d) / 'navigation' / 'web_token'
      token = load_token(path)
      self.assertGreaterEqual(len(token), 12)
      self.assertEqual(load_token(path), token)
      self.assertEqual(path.stat().st_mode & 0o777, 0o600)

  def test_web_url(self):
    with patch('openpilot.selfdrive.navigation.navweb.wlan_address', return_value='172.20.10.2'):
      self.assertEqual(web_url('tok'), 'http://172.20.10.2:8099/?t=tok')
    with patch('openpilot.selfdrive.navigation.navweb.wlan_address', return_value=None):
      self.assertIsNone(web_url('tok'))


class TestServer(unittest.TestCase):
  def setUp(self):
    self.params = FakeParams()
    self.status = {'gps': {'fix': True, 'accuracy': 2.0}, 'instruction': None}
    self.server = make_server(0, self.params, 'secret-token', 'test-key', lambda: self.status)
    threading.Thread(target=self.server.serve_forever, kwargs={'poll_interval': 0.01}, daemon=True).start()
    self.addCleanup(self.server.server_close)
    self.addCleanup(self.server.shutdown)
    self.base = f'http://127.0.0.1:{self.server.server_port}'
    self.session = requests.Session()

  def login(self):
    response = self.session.get(self.base + '/?t=secret-token')
    self.assertEqual(response.status_code, 200)
    self.assertIn('導航', response.text)
    self.assertIn('navweb', self.session.cookies)

  def test_needs_the_token(self):
    self.assertEqual(requests.get(self.base + '/').status_code, 403)
    self.assertEqual(requests.get(self.base + '/?t=wrong').status_code, 403)
    self.assertEqual(requests.get(self.base + '/api/status').status_code, 403)
    self.assertEqual(requests.post(self.base + '/api/destination', json={'latitude': 25, 'longitude': 121}).status_code, 403)
    self.assertNotIn('NavDestination', self.params.values)

  def test_status_and_destination(self):
    self.login()
    status = self.session.get(self.base + '/api/status').json()
    self.assertEqual((status['destination'], status['gps']), (None, {'fix': True, 'accuracy': 2.0}))
    response = self.session.post(self.base + '/api/destination', json={'latitude': 25.0339, 'longitude': 121.5645, 'name': '台北101'})
    self.assertEqual(response.status_code, 200)
    self.assertEqual(self.params.values['NavDestination'], {'latitude': 25.0339, 'longitude': 121.5645, 'place_name': '台北101'})
    self.assertEqual(self.session.get(self.base + '/api/status').json()['destination']['place_name'], '台北101')
    self.assertEqual(self.session.delete(self.base + '/api/destination').status_code, 200)
    self.assertNotIn('NavDestination', self.params.values)

  def test_bad_destination(self):
    self.login()
    for body in ({'latitude': 95, 'longitude': 121}, {'latitude': 'x', 'longitude': 121}, {}):
      with self.subTest(body=body):
        self.assertEqual(self.session.post(self.base + '/api/destination', json=body).status_code, 400)
    self.assertEqual(self.session.post(self.base + '/api/destination', data='not json').status_code, 400)
    self.assertNotIn('NavDestination', self.params.values)

  def test_search_and_link(self):
    self.login()
    places = [{'name': '台北101', 'address': '', 'latitude': 25.0339, 'longitude': 121.5645}]
    with patch('openpilot.selfdrive.navigation.navweb.search_places', return_value=places) as search:
      self.assertEqual(self.session.post(self.base + '/api/search', json={'query': '101'}).json(), {'places': places})
    self.assertEqual(search.call_args.args[:2], ('101', 'test-key'))
    self.assertEqual(search.call_args.kwargs['near'], None)  # the status has no position
    link = self.session.post(self.base + '/api/link', json={'text': '25.1, 121.2'}).json()
    self.assertEqual(link, {'places': [{'name': '25.10000, 121.20000', 'address': '', 'latitude': 25.1, 'longitude': 121.2}]})
    with patch('openpilot.selfdrive.navigation.navweb.search_places', side_effect=requests.HTTPError('403')):
      self.assertEqual(self.session.post(self.base + '/api/search', json={'query': '101'}).status_code, 502)

  def test_link_without_coordinates_searches_its_place(self):
    self.login()
    places = [{'name': 'x', 'address': '', 'latitude': 1.0, 'longitude': 2.0}]
    with patch('openpilot.selfdrive.navigation.navweb.resolve_link', return_value=(None, 'Taipei 101')), \
         patch('openpilot.selfdrive.navigation.navweb.search_places', return_value=places) as search:
      self.assertEqual(self.session.post(self.base + '/api/link', json={'text': 'https://maps.app.goo.gl/x'}).json(), {'places': places})
    self.assertEqual(search.call_args.args[0], 'Taipei 101')

  def test_unknown_path(self):
    self.login()
    self.assertEqual(self.session.get(self.base + '/nope').status_code, 404)
    self.assertEqual(json.loads(self.session.get(self.base + '/nope').text)['error'], 'not found')


if __name__ == '__main__':
  unittest.main()
