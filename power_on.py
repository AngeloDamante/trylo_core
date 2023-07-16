#!/usr/bin/env python

import os
import time
import argparse
from src.trylo_gpio.trylo_gpio.Trylo import Trylo
from src.trylo_gpio.trylo_gpio.names import (
    LIGHT_FRONT_LEFT,
    LIGHT_FRONT_RIGHT,
    LIGHT_MIDDLE_LEFT,
    LIGHT_MIDDLE_RIGHT
)

robot = Trylo()
parser = argparse.ArgumentParser()
parser.add_argument("-B", "--build", type=str, default="yes")
args = parser.parse_args()

build_flag = args.build
if build_flag == "yes": os.system("make clean")

for i in range(4):
    if i == 0:
        robot.set_underlight(LIGHT_MIDDLE_LEFT, (204, 0, 204))
        if build_flag == "yes": os.system("make build_gpio")
        time.sleep(0.3)
        
    elif i == 1:
        robot.set_underlight(LIGHT_FRONT_LEFT, (102, 0, 204))
        if build_flag == "yes": os.system("make build_vision")
        time.sleep(0.3)
        
    elif i == 2:
        robot.set_underlight(LIGHT_FRONT_RIGHT, (0, 102, 204))
        if build_flag == "yes": os.system("make build_control")
        time.sleep(0.3)
        
    elif i == 3:
        robot.set_underlight(LIGHT_MIDDLE_RIGHT, (0, 204, 204))
        if build_flag == "yes": os.system("make build_launch")
        time.sleep(0.3)
        
del robot