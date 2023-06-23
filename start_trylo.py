#!/usr/bin/env python

from src.trylo_gpio.trylo_gpio.Trylo import Trylo
import time

GRAY_COLOR = (127, 127, 127)
NUM_HELLOS = 5

robot = Trylo()
for _ in range(NUM_HELLOS):
    robot.fill_underlighting(GRAY_COLOR, show=True)
    time.sleep(0.2)
    robot.clear_underlighting(show=True)
    time.sleep(0.2)