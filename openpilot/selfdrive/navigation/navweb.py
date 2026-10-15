#!/usr/bin/env python3
"""A phone page on the device for the navigation destination.

The phone shares its hotspot with the device and opens http://<device>:8099/?t=<token> from the QR code in the
developer settings; the token then lives in a cookie. The page searches Google Places, takes a Google Maps share
link or coordinates, and sets or clears NavDestination.
"""
import hmac
import json
import os
import re
import secrets
import subprocess
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from urllib.parse import parse_qs, unquote_plus, urlparse

import requests

import openpilot.cereal.messaging as messaging
from openpilot.common.hardware import PC
from openpilot.common.hardware.hw import Paths
from openpilot.common.params import Params
from openpilot.common.swaglog import cloudlog
from openpilot.selfdrive.navigation.google import load_api_key
from openpilot.selfdrive.navigation.helpers import latest_gps_source

PORT = 8099
TOKEN_PATH = Path(Paths.comma_home() if PC else '/data') / 'navigation' / 'web_token'
PLACES_URL = 'https://places.googleapis.com/v1/places:searchText'
PLACES_FIELDS = 'places.displayName,places.formattedAddress,places.location'
SEARCH_RADIUS = 50000.  # m around the car that places are biased to
MAX_BODY = 10000  # bytes
COOKIE = 'navweb'
INSTRUCTION_TIMEOUT = 3.  # s

MAPS_HOSTS = {'maps.app.goo.gl', 'goo.gl', 'maps.google.com', 'maps.google.com.tw'}
GOOGLE_HOST = re.compile(r'(www\.)?google\.com(\.tw)?')
PIN = re.compile(r'!3d(-?\d+(?:\.\d+)?)!4d(-?\d+(?:\.\d+)?)')
VIEW = re.compile(r'@(-?\d+(?:\.\d+)?),(-?\d+(?:\.\d+)?)')
PAIR = re.compile(r'\s*(-?\d+(?:\.\d+)?)\s*,\s*(-?\d+(?:\.\d+)?)\s*')
PLACE = re.compile(r'/maps/place/([^/@?]+)')


def _valid(lat: float, lon: float) -> tuple[float, float] | None:
  return (lat, lon) if -90 <= lat <= 90 and -180 <= lon <= 180 else None


def parse_coordinates(text: str) -> tuple[float, float] | None:
  """(latitude, longitude) from coordinates or a Google Maps URL: its pin, its q/query/ll parameter, else its view."""
  if m := PIN.search(text):
    return _valid(float(m[1]), float(m[2]))
  query = parse_qs(urlparse(text).query)
  for key in ('q', 'query', 'll', 'destination'):
    if key in query and (m := PAIR.fullmatch(query[key][0])):
      return _valid(float(m[1]), float(m[2]))
  if m := VIEW.search(text):
    return _valid(float(m[1]), float(m[2]))
  if m := PAIR.fullmatch(text):
    return _valid(float(m[1]), float(m[2]))
  return None


def place_name(url: str) -> str:
  m = PLACE.search(url)
  return unquote_plus(m[1]) if m else ''


def is_maps_url(url: str) -> bool:
  """Only Google Maps links are followed, so the device never fetches an address from the page."""
  parsed = urlparse(url.strip())
  host = (parsed.hostname or '').lower()
  if parsed.scheme not in ('http', 'https'):
    return False
  return host in MAPS_HOSTS or (GOOGLE_HOST.fullmatch(host) is not None and parsed.path.startswith('/maps'))


def resolve_link(text: str) -> tuple[tuple[float, float] | None, str]:
  """(coordinates or None, place name) of coordinates or a Google Maps link, following a share link's redirects."""
  text = text.strip()
  if coordinates := parse_coordinates(text):
    return coordinates, place_name(text)
  if not is_maps_url(text):
    return None, ''
  response = requests.get(text, allow_redirects=True, timeout=10, stream=True)
  response.close()
  if not is_maps_url(response.url):
    return None, ''
  return parse_coordinates(response.url), place_name(response.url)


def search_places(query: str, api_key: str, near: tuple[float, float] | None = None) -> list[dict]:
  """Google Places text search in Traditional Chinese, biased to the area around near (latitude, longitude)."""
  body = {'textQuery': query, 'languageCode': 'zh-TW', 'regionCode': 'TW', 'maxResultCount': 8}
  if near is not None:
    body['locationBias'] = {'circle': {'center': {'latitude': near[0], 'longitude': near[1]}, 'radius': SEARCH_RADIUS}}
  response = requests.post(PLACES_URL, json=body, headers={'X-Goog-Api-Key': api_key, 'X-Goog-FieldMask': PLACES_FIELDS}, timeout=10)
  response.raise_for_status()
  return [{'name': p.get('displayName', {}).get('text', ''), 'address': p.get('formattedAddress', ''),
           'latitude': p['location']['latitude'], 'longitude': p['location']['longitude']}
          for p in response.json().get('places', []) if 'location' in p]


def load_token(path: Path = TOKEN_PATH) -> str:
  """The page's access token, made once and kept on the device."""
  try:
    return path.read_text().strip()
  except FileNotFoundError:
    pass
  path.parent.mkdir(parents=True, exist_ok=True)
  token = secrets.token_urlsafe(12)
  fd = os.open(path, os.O_WRONLY | os.O_CREAT | os.O_EXCL, 0o600)
  with os.fdopen(fd, 'w') as f:
    f.write(token)
  return token


def wlan_address() -> str | None:
  """The device's IPv4 address on Wi-Fi, which is the phone's hotspot while driving."""
  try:
    out = subprocess.run(['ip', '-4', '-o', 'addr', 'show', 'dev', 'wlan0'], capture_output=True, text=True, timeout=2).stdout
  except (OSError, subprocess.SubprocessError):
    return None
  m = re.search(r'inet (\d+\.\d+\.\d+\.\d+)/', out)
  return m[1] if m else None


def web_url(token: str, port: int = PORT) -> str | None:
  address = wlan_address()
  return f'http://{address}:{port}/?t={token}' if address else None


class Status:
  """GPS and the next instruction for the page, read from messaging by whichever request thread asks."""

  def __init__(self):
    self.sm = messaging.SubMaster(['gpsLocation', 'gpsLocationExternal', 'navInstruction'])
    self.lock = threading.Lock()

  def __call__(self) -> dict:
    with self.lock:
      self.sm.update(0)
      source = latest_gps_source(self.sm)
      gps = {'fix': False}
      if source is not None:
        g = self.sm[source]
        gps = {'fix': True, 'accuracy': round(g.horizontalAccuracy, 1), 'latitude': g.latitude, 'longitude': g.longitude}
      instruction = None
      if self.sm.valid['navInstruction'] and time.monotonic() - self.sm.recv_time['navInstruction'] < INSTRUCTION_TIMEOUT:
        i = self.sm['navInstruction']
        instruction = {'text': i.maneuverPrimaryText, 'distance': round(i.maneuverDistance),
                       'remaining_distance': round(i.distanceRemaining), 'remaining_time': round(i.timeRemaining)}
      return {'gps': gps, 'instruction': instruction}


class Handler(BaseHTTPRequestHandler):
  server: 'NavWebServer'

  def log_message(self, format: str, *args: object) -> None:  # noqa: A002  # stdlib override
    pass

  def send_json(self, code: int, body: dict):
    data = json.dumps(body, ensure_ascii=False).encode()
    self.send_response(code)
    self.send_header('Content-Type', 'application/json; charset=utf-8')
    self.send_header('Content-Length', str(len(data)))
    self.send_header('Cache-Control', 'no-store')
    self.end_headers()
    self.wfile.write(data)

  def send_page(self, code: int, html: str, cookie: str | None = None):
    data = html.encode()
    self.send_response(code)
    self.send_header('Content-Type', 'text/html; charset=utf-8')
    self.send_header('Content-Length', str(len(data)))
    self.send_header('Cache-Control', 'no-store')
    if cookie is not None:
      self.send_header('Set-Cookie', f'{COOKIE}={cookie}; Path=/; HttpOnly; SameSite=Strict; Max-Age=31536000')
    self.end_headers()
    self.wfile.write(data)

  def query_token(self) -> str:
    return parse_qs(urlparse(self.path).query).get('t', [''])[0]

  def cookie_token(self) -> str:
    for part in self.headers.get('Cookie', '').split(';'):
      name, _, value = part.strip().partition('=')
      if name == COOKIE:
        return value
    return ''

  def authorized(self) -> bool:
    return any(hmac.compare_digest(t.encode(), self.server.token.encode()) for t in (self.query_token(), self.cookie_token()) if t)

  def body(self) -> dict | None:
    length = int(self.headers.get('Content-Length') or 0)
    if not 0 < length <= MAX_BODY:
      return None
    try:
      body = json.loads(self.rfile.read(length))
    except ValueError:
      return None
    return body if isinstance(body, dict) else None

  def near(self) -> tuple[float, float] | None:
    gps = self.server.status().get('gps', {})
    return (gps['latitude'], gps['longitude']) if 'latitude' in gps else None

  def do_GET(self):
    path = urlparse(self.path).path
    if not self.authorized():
      if path == '/':
        self.send_page(403, DENIED_PAGE)
      else:
        self.send_json(403, {'error': 'forbidden'})
      return
    if path == '/':
      self.send_page(200, PAGE, cookie=self.server.token)
    elif path == '/api/status':
      self.send_json(200, {'destination': self.server.params.get('NavDestination'), **self.server.status()})
    else:
      self.send_json(404, {'error': 'not found'})

  def do_POST(self):
    path = urlparse(self.path).path
    if not self.authorized():
      self.send_json(403, {'error': 'forbidden'})
      return
    body = self.body()
    if body is None:
      self.send_json(400, {'error': 'bad request'})
      return
    try:
      if path == '/api/destination':
        self.set_destination(body)
      elif path == '/api/search':
        self.search(body)
      elif path == '/api/link':
        self.link(body)
      else:
        self.send_json(404, {'error': 'not found'})
    except requests.RequestException:
      cloudlog.exception('navweb: Google request failed')
      self.send_json(502, {'error': 'Google request failed'})

  def do_DELETE(self):
    if not self.authorized():
      self.send_json(403, {'error': 'forbidden'})
    elif urlparse(self.path).path == '/api/destination':
      self.server.params.remove('NavDestination')
      self.send_json(200, {'ok': True})
    else:
      self.send_json(404, {'error': 'not found'})

  def set_destination(self, body: dict):
    try:
      coordinates = _valid(float(body['latitude']), float(body['longitude']))
    except (KeyError, TypeError, ValueError):
      coordinates = None
    if coordinates is None:
      self.send_json(400, {'error': 'bad coordinates'})
      return
    destination = {'latitude': coordinates[0], 'longitude': coordinates[1]}
    if name := str(body.get('name') or '')[:100]:
      destination['place_name'] = name
    self.server.params.put('NavDestination', destination)
    self.send_json(200, {'ok': True})

  def search(self, body: dict):
    query = str(body.get('query') or '').strip()[:200]
    if not query:
      self.send_json(400, {'error': 'empty query'})
    elif self.server.api_key is None:
      self.send_json(503, {'error': 'no Google Maps API key on the device'})
    else:
      self.send_json(200, {'places': search_places(query, self.server.api_key, near=self.near())})

  def link(self, body: dict):
    coordinates, name = resolve_link(str(body.get('text') or '')[:2000])
    if coordinates is not None:
      lat, lon = coordinates
      self.send_json(200, {'places': [{'name': name or f'{lat:.5f}, {lon:.5f}', 'address': '', 'latitude': lat, 'longitude': lon}]})
    elif name and self.server.api_key is not None:
      self.send_json(200, {'places': search_places(name, self.server.api_key, near=self.near())})
    else:
      self.send_json(200, {'places': []})


class NavWebServer(ThreadingHTTPServer):
  daemon_threads = True

  def __init__(self, port: int, params, token: str, api_key: str | None, status):
    super().__init__(('', port), Handler)
    self.params, self.token, self.api_key, self.status = params, token, api_key, status


def make_server(port: int, params, token: str, api_key: str | None, status) -> NavWebServer:
  return NavWebServer(port, params, token, api_key, status)


DENIED_PAGE = """<!doctype html><html lang="zh-Hant"><meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1"><title>導航</title>
<body style="font-family:system-ui;padding:24px">請用手機掃描裝置「開發者」設定裡的導航 QR code。</body></html>"""

PAGE = """<!doctype html>
<html lang="zh-Hant">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>導航</title>
<style>
  :root { --bg: #f4f4f2; --card: #fff; --text: #1d1d1b; --muted: #6b6b66; --line: #ddddd8; --accent: #1f6feb; --warn: #b54708; }
  @media (prefers-color-scheme: dark) {
    :root { --bg: #151514; --card: #222220; --text: #ececea; --muted: #a3a39e; --line: #383835; --accent: #4c8dff; --warn: #f0a35e; }
  }
  * { box-sizing: border-box; }
  body { margin: 0; background: var(--bg); color: var(--text); font: 16px/1.45 system-ui, -apple-system, "PingFang TC", sans-serif; }
  main { max-width: 560px; margin: 0 auto; padding: 16px; }
  h1 { font-size: 20px; margin: 4px 0 12px; }
  section { background: var(--card); border: 1px solid var(--line); border-radius: 12px; padding: 14px; margin-bottom: 12px; }
  h2 { font-size: 14px; color: var(--muted); font-weight: 600; margin: 0 0 8px; }
  .big { font-size: 18px; font-weight: 600; }
  .muted { color: var(--muted); font-size: 14px; }
  .warn { color: var(--warn); font-size: 14px; }
  form { display: flex; gap: 8px; }
  input, textarea { flex: 1; min-width: 0; font: inherit; padding: 10px; border-radius: 8px; border: 1px solid var(--line);
                    background: var(--bg); color: var(--text); }
  textarea { resize: vertical; min-height: 44px; }
  button { font: inherit; padding: 10px 14px; border-radius: 8px; border: 0; background: var(--accent); color: #fff; }
  button.plain { background: transparent; color: var(--accent); border: 1px solid var(--line); }
  ul { list-style: none; margin: 10px 0 0; padding: 0; }
  li { display: flex; gap: 10px; align-items: center; justify-content: space-between; padding: 10px 0; border-top: 1px solid var(--line); }
  li div { min-width: 0; }
  #error { color: var(--warn); }
</style>
</head>
<body>
<main>
  <h1>導航</h1>
  <section>
    <h2>目的地</h2>
    <div id="destination" class="big">未設定</div>
    <div id="destination-detail" class="muted"></div>
    <p><button id="clear" class="plain" hidden>清除目的地</button></p>
  </section>
  <section>
    <h2>下一個指示</h2>
    <div id="instruction" class="big">沒有路線</div>
    <div id="remaining" class="muted"></div>
    <div id="gps" class="muted"></div>
  </section>
  <section>
    <h2>搜尋地點</h2>
    <form id="search-form"><input id="query" placeholder="地名或地址" enterkeyhint="search"><button>搜尋</button></form>
  </section>
  <section>
    <h2>Google Maps 分享連結或座標</h2>
    <form id="link-form"><textarea id="link" placeholder="貼上分享連結，或 25.0339, 121.5645"></textarea><button>解析</button></form>
  </section>
  <section id="results-card" hidden>
    <h2>結果</h2>
    <div id="error"></div>
    <ul id="results"></ul>
  </section>
</main>
<script>
const $ = (id) => document.getElementById(id);
function meters(m) { return m >= 1000 ? (m / 1000).toFixed(1) + ' 公里' : Math.round(m) + ' 公尺'; }
async function api(method, path, body) {
  const r = await fetch(path, { method, headers: body ? { 'Content-Type': 'application/json' } : {},
                                body: body ? JSON.stringify(body) : undefined, credentials: 'same-origin' });
  const data = await r.json().catch(() => ({}));
  if (!r.ok) throw new Error(data.error || ('HTTP ' + r.status));
  return data;
}
async function refresh() {
  try {
    const s = await api('GET', '/api/status');
    const d = s.destination;
    $('destination').textContent = d ? (d.place_name || '已設定') : '未設定';
    $('destination-detail').textContent = d ? d.latitude.toFixed(5) + ', ' + d.longitude.toFixed(5) : '';
    $('clear').hidden = !d;
    const i = s.instruction;
    $('instruction').textContent = i ? (i.text || '直行') + '（' + meters(i.distance) + '）' : (d ? '等待路線（上路後開始）' : '沒有路線');
    $('remaining').textContent = i ? '剩餘 ' + meters(i.remaining_distance) + '，約 ' + Math.max(1, Math.round(i.remaining_time / 60)) + ' 分鐘' : '';
    const g = s.gps;
    $('gps').textContent = !g.fix ? 'GPS 尚未定位' : 'GPS 精度 ' + g.accuracy + ' 公尺';
    $('gps').className = (!g.fix || g.accuracy > 5) ? 'warn' : 'muted';
    if (g.fix && g.accuracy > 5) $('gps').textContent += '（超過 5 公尺時不會規劃路線）';
  } catch (e) { $('gps').textContent = '無法連線到裝置：' + e.message; $('gps').className = 'warn'; }
}
function show(places, error) {
  $('results-card').hidden = false;
  $('error').textContent = error || (places.length ? '' : '找不到地點');
  const list = $('results');
  list.replaceChildren();
  for (const p of places) {
    const li = document.createElement('li');
    const text = document.createElement('div');
    const name = document.createElement('div');
    name.textContent = p.name;
    const address = document.createElement('div');
    address.className = 'muted';
    address.textContent = p.address || (p.latitude.toFixed(5) + ', ' + p.longitude.toFixed(5));
    text.append(name, address);
    const button = document.createElement('button');
    button.textContent = '設為目的地';
    button.onclick = async () => {
      try { await api('POST', '/api/destination', { latitude: p.latitude, longitude: p.longitude, name: p.name });
            $('results-card').hidden = true; refresh(); }
      catch (e) { $('error').textContent = '設定失敗：' + e.message; }
    };
    li.append(text, button);
    list.append(li);
  }
}
$('search-form').onsubmit = async (e) => {
  e.preventDefault();
  const query = $('query').value.trim();
  if (!query) return;
  try { show((await api('POST', '/api/search', { query })).places); } catch (err) { show([], '搜尋失敗：' + err.message); }
};
$('link-form').onsubmit = async (e) => {
  e.preventDefault();
  const text = $('link').value.trim();
  if (!text) return;
  try { show((await api('POST', '/api/link', { text })).places); } catch (err) { show([], '解析失敗：' + err.message); }
};
$('clear').onclick = async () => { await api('DELETE', '/api/destination'); refresh(); };
refresh();
setInterval(refresh, 2000);
</script>
</body>
</html>
"""


def main():
  token, api_key = load_token(), load_api_key()
  if api_key is None:
    cloudlog.error('navweb: no Google Maps API key, search is off')
  server = make_server(PORT, Params(), token, api_key, Status())
  cloudlog.warning(f'navweb serving on port {PORT}')
  server.serve_forever()


if __name__ == '__main__':
  main()
