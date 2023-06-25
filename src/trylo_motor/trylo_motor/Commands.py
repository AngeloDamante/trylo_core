"""Define Commands to semplify msg"""

from enum import Enum

class Commands(Enum):
    FORWARD = 1
    BACKWARD = 2
    TURN_RIGHT = 3
    TURN_LEFT = 4
    CURVE_RIGHT = 5
    CURVE_LEFT = 6
    STOP = 7
    COAST = 8