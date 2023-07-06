"""Definitions of type utilities"""

from enum import Enum

class MARKER(Enum):
    enable_tracker: int = 1
    disable_tracker: int = 0
    target_0: int = 128
    target_1: int = 129


# class COMMAND(Enum):
#     forward: str = "FORWARD"
#     stop: str = "STOP"
#     turn_left: str = "TURN_LEFT"
#     turn_right: str = "TURN_RIGHT"


class STATE(Enum):
    off: str = "OFF"
    enable: str = "ENABLE"
    follow: str = "FOLLOW"

TARGETS = [MARKER.target_0, MARKER.target_1]