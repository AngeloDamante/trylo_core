#!/usr/bin/env python

import os
from src.trylo_gpio.trylo_gpio.Trylo import Trylo
from src.trylo_gpio.trylo_gpio.names import (
    LIGHT_FRONT_LEFT,
    LIGHT_FRONT_RIGHT,
    LIGHT_MIDDLE_LEFT,
    LIGHT_MIDDLE_RIGHT
)

robot = Trylo()

for i in range(4):
    if i == 0:
        robot.set_underlight(LIGHT_MIDDLE_LEFT, (204, 0, 204))
        os.system("make build_gpio")
    elif i == 1:
        robot.set_underlight(LIGHT_FRONT_LEFT, (102, 0, 204))
        os.system("make build_vision")
    elif i == 2:
        robot.set_underlight(LIGHT_FRONT_RIGHT, (0, 102, 204))
        os.system("make build_control")
    elif i == 3:
        robot.set_underlight(LIGHT_MIDDLE_RIGHT, (0, 204, 204))
        os.system("make build_launch")
        
del robot