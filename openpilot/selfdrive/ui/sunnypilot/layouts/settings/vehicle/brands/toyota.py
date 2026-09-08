"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""
from opendbc.sunnypilot.car.toyota.auto_brake_hold import is_auto_brake_hold_available
from openpilot.selfdrive.ui.sunnypilot.layouts.settings.vehicle.brands.base import BrandSettings
from openpilot.selfdrive.ui.ui_state import ui_state
from openpilot.system.ui.lib.application import gui_app
from openpilot.system.ui.lib.multilang import tr, tr_noop
from openpilot.system.ui.widgets import DialogResult
from openpilot.system.ui.widgets.confirm_dialog import ConfirmDialog
from openpilot.system.ui.sunnypilot.widgets.list_view import toggle_item_sp


ONROAD_ONLY_DESCRIPTION = tr_noop("Start the vehicle to check vehicle compatibility.")
SNG_HACK_UNAVAILABLE = tr_noop("sunnypilot Longitudinal Control must be available and enabled for your vehicle to use this feature.")
AUTO_BRAKE_HOLD_UNAVAILABLE = tr_noop("This feature is only available on Toyota/Lexus vehicles with camera based ACC (TSS2/LSS2), " +
                                      "and requires sunnypilot Longitudinal Control to be available and enabled for your vehicle.")

DESCRIPTIONS = {
  'enforce_stock_longitudinal': tr_noop(
    'sunnypilot will not take over control of gas and brakes. Factory Toyota longitudinal control will be used.'
  ),
  'stop_and_go_hack': tr_noop(
    'sunnypilot will allow some Toyota/Lexus cars to auto resume during stop and go traffic. ' +
    'This feature is only applicable to certain models that are able to use longitudinal control. This is an alpha feature. Use at your own risk.'
  ),
  'auto_brake_hold': tr_noop(
    'Hold the brake pedal for 500 ms once the vehicle has come to a stop to engage brake hold. ' +
    'Pressing the accelerator, vehicle movement, engaging ACC, turning cruise main off, or shifting into park or reverse releases it. ' +
    'This is an alpha feature. Use at your own risk.'
  )
}


class ToyotaSettings(BrandSettings):
  def __init__(self):
    super().__init__()

    self.enforce_stock_longitudinal = toggle_item_sp(
      lambda: tr("Enforce Factory Longitudinal Control"),
      description=lambda: tr(DESCRIPTIONS["enforce_stock_longitudinal"]),
      initial_state=ui_state.params.get_bool("ToyotaEnforceStockLongitudinal"),
      callback=self._on_enable_enforce_stock_longitudinal,
      enabled=lambda: not ui_state.engaged,
    )

    self.stop_and_go_hack = toggle_item_sp(
      lambda: tr("Stop and Go Hack (Alpha)"),
      description=lambda: tr(DESCRIPTIONS["stop_and_go_hack"]),
      initial_state=ui_state.params.get_bool("ToyotaStopAndGoHack"),
      callback=self._on_enable_stop_and_go_hack,
      enabled=lambda: not ui_state.engaged,
    )

    self.auto_brake_hold = toggle_item_sp(
      lambda: tr("Automatic Brake Hold"),
      description=lambda: tr(DESCRIPTIONS["auto_brake_hold"]),
      initial_state=ui_state.params.get_bool("ToyotaAutoHold"),
      callback=self._on_enable_auto_brake_hold,
      enabled=lambda: not ui_state.engaged,
    )

    self.items = [
      self.enforce_stock_longitudinal,
      self.stop_and_go_hack,
      self.auto_brake_hold,
    ]

  def _on_enable_enforce_stock_longitudinal(self, state: bool):
    if state:
      def confirm_callback(result: int):
        if result == DialogResult.CONFIRM:
          ui_state.params.put_bool("ToyotaEnforceStockLongitudinal", True)
          if ui_state.params.get_bool("AlphaLongitudinalEnabled"):
            ui_state.params.put_bool("AlphaLongitudinalEnabled", False)
          ui_state.params.put_bool("ToyotaStopAndGoHack", False)
          self.stop_and_go_hack.action_item.set_state(False)
          ui_state.params.put_bool("ToyotaAutoHold", False)
          self.auto_brake_hold.action_item.set_state(False)
          ui_state.params.put_bool("OnroadCycleRequested", True)
        else:
          self.enforce_stock_longitudinal.action_item.set_state(False)

      content = (f"<h1>{self.enforce_stock_longitudinal.title}</h1><br>" +
                 f"<p>{self.enforce_stock_longitudinal.description}</p>")

      dlg = ConfirmDialog(content, tr("Enable"), rich=True, callback=confirm_callback)
      gui_app.push_widget(dlg)

    else:
      ui_state.params.put_bool("ToyotaEnforceStockLongitudinal", False)
      ui_state.params.put_bool("OnroadCycleRequested", True)

  def _on_enable_stop_and_go_hack(self, state: bool):
    if state:
      def confirm_callback(result: int):
        if result == DialogResult.CONFIRM:
          ui_state.params.put_bool("ToyotaStopAndGoHack", True)
          ui_state.params.put_bool("OnroadCycleRequested", True)
        else:
          self.stop_and_go_hack.action_item.set_state(False)

      content = (f"<h1>{self.stop_and_go_hack.title}</h1><br>" +
                 f"<p>{self.stop_and_go_hack.description}</p>")

      dlg = ConfirmDialog(content, tr("Enable"), rich=True, callback=confirm_callback)
      gui_app.push_widget(dlg)

    else:
      ui_state.params.put_bool("ToyotaStopAndGoHack", False)
      ui_state.params.put_bool("OnroadCycleRequested", True)

  def _on_enable_auto_brake_hold(self, state: bool):
    if state:
      def confirm_callback(result: int):
        if result == DialogResult.CONFIRM:
          ui_state.params.put_bool("ToyotaAutoHold", True)
          ui_state.params.put_bool("OnroadCycleRequested", True)
        else:
          self.auto_brake_hold.action_item.set_state(False)

      content = (f"<h1>{self.auto_brake_hold.title}</h1><br>" +
                 f"<p>{self.auto_brake_hold.description}</p>")

      dlg = ConfirmDialog(content, tr("Enable"), rich=True, callback=confirm_callback)
      gui_app.push_widget(dlg)

    else:
      ui_state.params.put_bool("ToyotaAutoHold", False)
      ui_state.params.put_bool("OnroadCycleRequested", True)

  @staticmethod
  def _update_description(item, description: str, unavailable_reason: str) -> None:
    new_desc = ("<b>" + unavailable_reason + "</b>\n\n" + description) if unavailable_reason else description
    if item.description != new_desc:
      item.set_description(new_desc)
      if unavailable_reason:
        item.show_description(True)

  def update_settings(self):
    if ui_state.CP is not None:
      longitudinal = ui_state.CP.openpilotLongitudinalControl
      enforce_stock = self.enforce_stock_longitudinal.action_item.get_state()

      sng_available = longitudinal and not enforce_stock
      self.stop_and_go_hack.action_item.set_enabled(sng_available and not ui_state.engaged)
      if not sng_available:
        self.stop_and_go_hack.action_item.set_state(False)
      self._update_description(self.stop_and_go_hack, tr(DESCRIPTIONS["stop_and_go_hack"]),
                               "" if sng_available else tr(SNG_HACK_UNAVAILABLE))

      # auto brake hold commands the vehicle's pre-collision braking, so clear it outright on
      # platforms that cannot support it rather than leaving a stale param behind
      auto_brake_hold_available = is_auto_brake_hold_available(ui_state.CP) and not enforce_stock
      self.auto_brake_hold.action_item.set_enabled(auto_brake_hold_available and not ui_state.engaged)
      if not auto_brake_hold_available:
        self.auto_brake_hold.action_item.set_state(False)
        if ui_state.params.get_bool("ToyotaAutoHold"):
          ui_state.params.put_bool("ToyotaAutoHold", False)
      self._update_description(self.auto_brake_hold, tr(DESCRIPTIONS["auto_brake_hold"]),
                               "" if auto_brake_hold_available else tr(AUTO_BRAKE_HOLD_UNAVAILABLE))
    else:
      # compatibility is unknown offroad: disable the toggles but keep whatever the user stored
      self.stop_and_go_hack.action_item.set_enabled(False)
      self._update_description(self.stop_and_go_hack, tr(DESCRIPTIONS["stop_and_go_hack"]), tr(ONROAD_ONLY_DESCRIPTION))

      self.auto_brake_hold.action_item.set_enabled(False)
      self._update_description(self.auto_brake_hold, tr(DESCRIPTIONS["auto_brake_hold"]), tr(ONROAD_ONLY_DESCRIPTION))
