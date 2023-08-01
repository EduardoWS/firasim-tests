import numpy as np
from parameters import *
from enum import Enum

def norm_pos(pos):
    return np.clip(
        pos / max_pos,
        -NORM_BOUNDS,
        NORM_BOUNDS
    )

def norm_v(v):
    return np.clip(
        v / max_v,
        -NORM_BOUNDS,
        NORM_BOUNDS
    )

def norm_w(w):
    return np.clip(
        w / max_w,
        -NORM_BOUNDS,
        NORM_BOUNDS
    )

class TeamColor(Enum):
    BLUE = 0
    YELLOW = 1

class TeamSide(Enum):
    RIGHT = 0
    LEFT = 1