from dataclasses import dataclass
import numpy as np

@dataclass
class MovingObstacles:
    pos: np.ndarray
    vel: np.ndarray
    r: float

