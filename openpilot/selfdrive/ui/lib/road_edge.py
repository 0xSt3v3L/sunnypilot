import numpy as np

ROAD_EDGE_RGB = (255, 0, 0)
MAX_ALPHA = 0.7  # same cap as the mici lane lines


def road_edge_rgba(edge_std: float, disengaged: bool) -> tuple[int, int, int, int]:
  """Road edges are red while engaged, as on the comma 3 UI, and black when disengaged. Alpha follows model confidence."""
  alpha = int(np.clip(1.0 - edge_std, 0.0, MAX_ALPHA) * 255)
  rgb = (0, 0, 0) if disengaged else ROAD_EDGE_RGB
  return (*rgb, alpha)
