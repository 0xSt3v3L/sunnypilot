from openpilot.common.params import Params
from openpilot.system.ui.widgets.scroller import NavScroller
from openpilot.selfdrive.ui.lib.always_offroad import always_offroad_button, set_always_offroad, ENABLE_ONROAD, ENABLE_OFFROAD, DISABLE
from openpilot.selfdrive.ui.mici.widgets.button import BigButton, BigCircleButton
from openpilot.selfdrive.ui.mici.widgets.dialog import BigConfirmationDialog, BigDialog
from openpilot.selfdrive.ui.ui_state import ui_state
from openpilot.selfdrive.ui.mici.layouts.settings.toggles import TogglesLayoutMici
from openpilot.selfdrive.ui.mici.layouts.settings.network.network_layout import NetworkLayoutMici
from openpilot.selfdrive.ui.mici.layouts.settings.device.device_layout import DeviceLayoutMici
from openpilot.selfdrive.ui.mici.layouts.settings.developer import DeveloperLayoutMici
from openpilot.selfdrive.ui.mici.layouts.settings.software import SoftwareLayoutMici
from openpilot.selfdrive.ui.mici.layouts.settings.firehose import FirehoseLayout
from openpilot.system.ui.lib.application import gui_app, FontWeight


class SettingsBigButton(BigButton):
  def _get_label_font_size(self):
    return 64


class SettingsLayout(NavScroller):
  def __init__(self):
    super().__init__()
    self._params = Params()

    toggles_panel = TogglesLayoutMici()
    toggles_btn = SettingsBigButton("toggles", "", gui_app.texture("icons_mici/settings.png", 64, 64))
    toggles_btn.set_click_callback(lambda: gui_app.push_widget(toggles_panel))

    network_panel = NetworkLayoutMici()
    network_btn = SettingsBigButton("network", "", gui_app.texture("icons_mici/settings/network/wifi_strength_full.png", 76, 56))
    network_btn.set_click_callback(lambda: gui_app.push_widget(network_panel))

    self._device_panel = DeviceLayoutMici()
    self._device_button = SettingsBigButton("device", "", gui_app.texture("icons_mici/settings/device_icon.png", 72, 58))
    self._device_button.set_click_callback(lambda: gui_app.push_widget(self._device_panel))

    software_panel = SoftwareLayoutMici()
    software_btn = SettingsBigButton("software", "", gui_app.texture("icons_mici/settings/software.png", 64, 75))
    software_btn.set_click_callback(lambda: gui_app.push_widget(software_panel))

    developer_panel = DeveloperLayoutMici()
    developer_btn = SettingsBigButton("developer", "", gui_app.texture("icons_mici/settings/developer_icon.png", 64, 60))
    developer_btn.set_click_callback(lambda: gui_app.push_widget(developer_panel))

    firehose_panel = FirehoseLayout()
    firehose_btn = SettingsBigButton("firehose", "", gui_app.texture("icons_mici/settings/firehose.png", 52, 62))
    firehose_btn.set_click_callback(lambda: gui_app.push_widget(firehose_panel))

    # always offroad: only one of these buttons is visible at a time
    icon_enable_offroad = gui_app.texture("icons_mici/always_offroad.png", 110, 110)
    enable_offroad_btn_onroad = BigCircleButton(icon_enable_offroad, red=True)
    enable_offroad_btn_onroad.set_click_callback(lambda: self._handle_always_offroad(True))
    enable_offroad_btn_onroad.set_visible(lambda: always_offroad_button(ui_state.started, ui_state.always_offroad) == ENABLE_ONROAD)

    enable_offroad_btn_offroad = BigCircleButton(icon_enable_offroad, red=True)
    enable_offroad_btn_offroad.set_click_callback(lambda: self._handle_always_offroad(True))
    enable_offroad_btn_offroad.set_visible(lambda: always_offroad_button(ui_state.started, ui_state.always_offroad) == ENABLE_OFFROAD)

    disable_offroad_btn = BigCircleButton(gui_app.texture("icons_mici/disable_offroad.png", 110, 110))
    disable_offroad_btn.set_click_callback(lambda: self._handle_always_offroad(False))
    disable_offroad_btn.set_visible(lambda: always_offroad_button(ui_state.started, ui_state.always_offroad) == DISABLE)

    self._scroller.add_widgets([
      disable_offroad_btn,
      enable_offroad_btn_onroad,
      toggles_btn,
      network_btn,
      self._device_button,
      software_btn,
      firehose_btn,
      developer_btn,
      enable_offroad_btn_offroad,
    ])

    self._font_medium = gui_app.font(FontWeight.MEDIUM)

  def _handle_always_offroad(self, enable: bool):
    def set_status(status: bool):
      if set_always_offroad(self._params, status, ui_state.engaged):
        ui_state.always_offroad = status

    slider_icon = gui_app.texture("icons_mici/settings/device/lkas.png", 110, 110)
    if not enable:
      dlg = BigConfirmationDialog("slide to exit always offroad", slider_icon, confirm_callback=lambda: set_status(False))
    elif ui_state.engaged:
      dlg = BigDialog("disengage to enable always offroad", "")
    else:
      dlg = BigConfirmationDialog("slide to force offroad", slider_icon, red=True, confirm_callback=lambda: set_status(True))
    gui_app.push_widget(dlg)

  def show_pairing(self):
    gui_app.push_widget(self)
    # add settings in the back stack without showing its entrance animation.
    self._y_pos_filter.x = 0.0
    self.set_visible(lambda: self.enabled or self._device_panel.is_dismissing)
    self.set_shown_callback(self._scroll_to_device)
    gui_app.push_widget(self._device_panel)
    self._device_panel.set_shown_callback(self._on_pairing_shown)

  def _scroll_to_device(self):
    self._scroller._layout()
    offset = (self._device_button.rect.x + self._device_button.rect.width / 2) - (self._rect.x + self._rect.width / 2)
    self._scroller.scroll_to(offset, smooth=False)

  def _on_pairing_shown(self):
    self.set_visible(True)
    self._device_panel.scroll_to_pairing()
