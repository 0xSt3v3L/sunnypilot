from openpilot.selfdrive.ui.lib.navigation_page import page_lines
from openpilot.selfdrive.ui.mici.widgets.button import GreyBigButton
from openpilot.selfdrive.ui.mici.widgets.qr import QR
from openpilot.system.ui.widgets.scroller import NavScroller


class NavigationPage(NavScroller):
  """The QR code of navweb, the phone page that sets the destination, made when the page opens since the IP can change."""

  def __init__(self):
    super().__init__()
    url, text = page_lines()
    widgets = [QR(url), GreyBigButton("navigation page", "scan with the phone\nsharing its hotspot")] if url else []
    self._scroller.add_widgets(widgets + [GreyBigButton("", text)])
