TURN_TEXT = {"turnLeft": "left", "turnRight": "right"}


def debug_lines(ss, ss_sp, car_state, radar_state, model_v2, model_data_sp) -> list[str]:
  """Onroad debug overlay text, shown with ui debug mode (ShowDebugInfo)."""
  mads = ss_sp.mads
  mads_state = str(mads.state) if mads.available else "n/a"
  lat_active = mads.active if mads.available else ss.active

  lead = radar_state.leadOne
  lead_text = f"lead {lead.dRel:.1f}m {lead.vRel:+.1f}" if lead.present else "lead --"

  x = model_v2.position.x
  plan_text = f"plan {x[-1]:.0f}m" if len(x) else "plan --"

  edge = model_data_sp.roadEdge
  return [
    f"mads {mads_state}  lat {'on' if lat_active else 'off'}",
    f"steer {car_state.steeringAngleDeg:+.1f}  trq {car_state.steeringTorque:+.0f}",
    lead_text,
    plan_text,
    f"edge L{int(edge.leftDetected)} {edge.leftProb:.2f}  R{int(edge.rightDetected)} {edge.rightProb:.2f}",
    f"turn {TURN_TEXT.get(str(model_data_sp.laneTurnDirection), '--')}",
  ]
