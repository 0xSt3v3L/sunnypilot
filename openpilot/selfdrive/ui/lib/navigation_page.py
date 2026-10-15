from openpilot.selfdrive.navigation.navweb import load_token, web_url


def page_lines() -> tuple[str | None, str]:
  """(the navweb URL for the QR code, or None off Wi-Fi; the text under it)."""
  url = web_url(load_token())
  if url is None:
    return None, "connect the device to the phone's hotspot first"
  return url, url.split('/?')[0]
