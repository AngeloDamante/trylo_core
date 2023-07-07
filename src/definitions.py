"""Definitions of type utilities"""

from enum import Enum


class MARKER(Enum):
    enable_tracker: int = 1
    disable_tracker: int = 0
    target_0: int = 128
    target_1: int = 129


class STATE(Enum):
    off: str = "OFF"
    enable: str = "ENABLE"
    follow: str = "FOLLOW"


class Directive(Enum):
    FORWARD: int = 1
    BACKWARD: int = 2
    TURN_RIGHT: int = 3
    TURN_LEFT: int = 4
    CURVE_RIGHT: int = 5
    CURVE_LEFT: int = 6
    STOP: int = 7
    COAST: int = 8


class Led(Enum):
    FILL: int = 1
    CLEAR: int = 2
    FRONT_SIDE: int = 3
    LEFT_SIDE: int = 4
    RIGHT_SIDE: int = 5


TARGETS = [MARKER.target_0, MARKER.target_1]
