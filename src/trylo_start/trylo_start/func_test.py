#!/usr/bin/env python

import rclpy
import time
import numpy as np
from trilobot import Trilobot as Trylo
from trilobot import NUM_MOTORS, NUM_UNDERLIGHTS, BUTTON_A, BUTTON_X, BUTTON_B, BUTTON_Y


def main(args=None):
    trylo = Trylo()

    print("Trilobot Function Test")

    time.sleep(2.0)
    trylo.fill_underlighting(127, 127, 127, show=False)
    for i in range(0, 10):
        print(i)
        trylo.show_underlighting()
        time.sleep(0.1)
        trylo.disable_underlighting()
        time.sleep(0.5)

    for led in range(NUM_UNDERLIGHTS):
        trylo.clear_underlighting(show=False)
        trylo.set_underlight(led, 255, 0, 0)
        time.sleep(0.1)
        trylo.clear_underlighting(show=False)
        trylo.set_underlight(led, 0, 255, 0)
        time.sleep(0.1)
        trylo.clear_underlighting(show=False)
        trylo.set_underlight(led, 0, 0, 255)
        time.sleep(0.1)

    trylo.clear_underlighting()

    h = 0
    v = 0
    spacing = 1.0 / NUM_UNDERLIGHTS

    a = 0
    b = 0
    x = 0
    y = 0
    while True:
        for led in range(NUM_UNDERLIGHTS):
            led_h = h + (led * spacing)
            if led_h >= 1.0:
                led_h -= 1.0
            trylo.set_underlight_hsv(led, led_h, 1, 1, show=False)

        trylo.show_underlighting()
        h += 0.5 / 360
        if h >= 1.0:
            h -= 1.0

        if trylo.read_button(BUTTON_A):
            a = min(a + 0.01, 1.0)
        else:
            a = max(a - 0.01, 0.0)
        trylo.set_button_led(BUTTON_A, a)

        if trylo.read_button(BUTTON_B):
            b = min(b + 0.01, 1.0)
        else:
            b = max(b - 0.01, 0.0)
        trylo.set_button_led(BUTTON_B, b)

        if trylo.read_button(BUTTON_X):
            x = min(x + 0.01, 1.0)
        else:
            x = max(x - 0.01, 0.0)
        trylo.set_button_led(BUTTON_X, x)

        if trylo.read_button(BUTTON_Y):
            y = min(y + 0.01, 1.0)
        else:
            y = max(y - 0.01, 0.0)
        trylo.set_button_led(BUTTON_Y, y)

        trylo.set_left_speed(a - b)
        trylo.set_right_speed(x - y)
        time.sleep(0.01)


if __name__ == "__main__":
    main()
