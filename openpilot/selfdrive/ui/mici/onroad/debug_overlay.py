import pyray as rl
from openpilot.selfdrive.ui.lib.debug_info import debug_lines
from openpilot.selfdrive.ui.ui_state import ui_state
from openpilot.system.ui.lib.application import gui_app, FontWeight
from openpilot.system.ui.lib.text_measure import measure_text_cached
from openpilot.system.ui.widgets import Widget

FONT_SIZE = 20
LINE_HEIGHT = 22
MARGIN = 16
PADDING = 6
TEXT_COLOR = rl.Color(255, 255, 255, 230)
BG_COLOR = rl.Color(0, 0, 0, 140)


class DebugOverlay(Widget):
  """Onroad debug values in the top right corner, shown with ui debug mode."""
  def __init__(self):
    super().__init__()
    self._font = gui_app.font(FontWeight.MEDIUM)

  def _render(self, rect: rl.Rectangle) -> None:
    sm = ui_state.sm
    lines = debug_lines(sm['selfdriveState'], sm['selfdriveStateSP'], sm['carState'], sm['radarState'],
                        sm['modelV2'], sm['modelDataSP'])
    width = max(measure_text_cached(self._font, line, FONT_SIZE).x for line in lines)
    x = rect.x + rect.width - MARGIN - width
    y = rect.y + MARGIN
    rl.draw_rectangle_rounded(rl.Rectangle(x - PADDING, y - PADDING, width + 2 * PADDING, len(lines) * LINE_HEIGHT + 2 * PADDING),
                              0.1, 4, BG_COLOR)
    for i, line in enumerate(lines):
      rl.draw_text_ex(self._font, line, rl.Vector2(x, y + i * LINE_HEIGHT), FONT_SIZE, 0, TEXT_COLOR)
