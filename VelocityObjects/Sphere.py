from dataclasses import dataclass
import numpy as np

@dataclass
class Sphere:
    pos: np.ndarray
    r: float