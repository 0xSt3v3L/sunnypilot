import re
import tempfile
import threading
import unittest
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from unittest.mock import MagicMock, patch

from openpilot.selfdrive.navigation.tiles import RemoteTiles

ARCHIVE = bytes(range(256)) * 4


class RangeHandler(BaseHTTPRequestHandler):
  def do_GET(self):
    self.server.paths.append(self.path)
    offset, end = map(int, re.fullmatch(r'bytes=(\d+)-(\d+)', self.headers['Range']).groups())
    data = ARCHIVE[offset:end + 1]
    self.send_response(206)
    self.send_header('Content-Range', f'bytes {offset}-{end}/{len(ARCHIVE)}')
    self.send_header('Content-Length', str(len(data)))
    self.send_header('ETag', '"public"')
    self.end_headers()
    self.wfile.write(data)

  def log_message(self, format: str, *args: object) -> None:  # noqa: A002  # stdlib override
    pass


class TestPublicTiles(unittest.TestCase):
  """A fixed archive URL, such as a public Protomaps build, read without the comma API."""

  def setUp(self):
    directory = tempfile.TemporaryDirectory()
    self.addCleanup(directory.cleanup)
    self.server = ThreadingHTTPServer(('127.0.0.1', 0), RangeHandler)
    self.server.paths = []
    threading.Thread(target=self.server.serve_forever, kwargs={'poll_interval': 0.01}, daemon=True).start()
    self.addCleanup(self.server.server_close)
    self.addCleanup(self.server.shutdown)
    self.api = MagicMock(side_effect=AssertionError('the comma API must not be used'))
    for name, value in [('Api', self.api), ('Params', MagicMock()), ('PC', True)]:
      self.enterContext(patch(f'openpilot.selfdrive.navigation.tiles.{name}', value))
    self.enterContext(patch('openpilot.selfdrive.navigation.tiles.Paths.comma_home', return_value=directory.name))

  def test_reads_ranges_from_the_url(self):
    tiles = RemoteTiles(url=f'http://127.0.0.1:{self.server.server_port}/20261014.pmtiles')
    self.addCleanup(tiles.close)
    self.assertEqual(tiles.size, len(ARCHIVE))
    self.assertEqual(tiles.read(300, 10), ARCHIVE[300:310])
    self.assertEqual(set(self.server.paths), {'/20261014.pmtiles'})
    self.api.assert_not_called()


if __name__ == '__main__':
  unittest.main()
