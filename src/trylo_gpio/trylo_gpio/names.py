"""Pins declaration by trilobot_schematic"""

# Underlighting LED locations
LIGHT_FRONT_RIGHT = 0
LIGHT_FRONT_LEFT = 1
LIGHT_MIDDLE_LEFT = 2
LIGHT_MIDDLE_RIGHT = 5
LIGHT_REAR_LEFT = 3
LIGHT_REAR_RIGHT = 4
NUM_UNDERLIGHTS = 6
NUM_SN3218_CHANNELS = 18

# Useful underlighting groups
LIGHTS_LEFT = (LIGHT_FRONT_LEFT, LIGHT_MIDDLE_LEFT, LIGHT_REAR_LEFT)
LIGHTS_RIGHT = (LIGHT_FRONT_RIGHT, LIGHT_MIDDLE_RIGHT, LIGHT_REAR_RIGHT)
LIGHTS_FRONT = (LIGHT_FRONT_LEFT, LIGHT_FRONT_RIGHT)
LIGHTS_MIDDLE = (LIGHT_MIDDLE_LEFT, LIGHT_MIDDLE_RIGHT)
LIGHTS_REAR = (LIGHT_REAR_LEFT, LIGHT_REAR_RIGHT)
LIGHTS_LEFT_DIAGONAL = (LIGHT_FRONT_LEFT, LIGHT_REAR_RIGHT)
LIGHTS_RIGHT_DIAGONAL = (LIGHT_FRONT_RIGHT, LIGHT_REAR_LEFT)

# Motor names
MOTOR_LEFT = 0
MOTOR_RIGHT = 1
NUM_MOTORS = 2
