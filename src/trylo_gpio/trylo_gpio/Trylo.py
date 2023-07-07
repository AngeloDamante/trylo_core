import time
import sn3218
import RPi.GPIO as GPIO
from typing import Union
from src.trylo_gpio.trylo_gpio.names import * 


class Trylo:
    # Onboard LEDs pins (next to each button)
    LED_A_PIN = 23
    LED_B_PIN = 22
    LED_X_PIN = 17
    LED_Y_PIN = 27

    # Motor driver pins, via DRV8833PWP Dual H-Bridge
    MOTOR_EN_PIN = 26
    MOTOR_LEFT_P = 8
    MOTOR_LEFT_N = 11
    MOTOR_RIGHT_P = 10
    MOTOR_RIGHT_N = 9

    # HC-SR04 Ultrasound pins
    ULTRA_TRIG_PIN = 13
    ULTRA_ECHO_PIN = 25

    # SN3218 LED Driver pin
    UNDERLIGHTING_EN_PIN = 7

    # Speed of sound is 343m/s which we need in cm/ns for our distance measure
    SPEED_OF_SOUND_CM_NS = 343 * 100 / 1E9  # 0.0000343 cm / ns

    # attributes
    leds: tuple = None
    led_pwm_mapping: dict = None
    motor_pwm_mapping: dict = None
    sn3218: any = None

    def __init__(self):
        """ Trylo Initialisation
        """
        GPIO.setwarnings(False)
        GPIO.setmode(GPIO.BCM)

        # Setup user LEDs
        GPIO.setup(self.LED_A_PIN, GPIO.OUT)
        GPIO.setup(self.LED_B_PIN, GPIO.OUT)
        GPIO.setup(self.LED_X_PIN, GPIO.OUT)
        GPIO.setup(self.LED_Y_PIN, GPIO.OUT)
        self.leds = (self.LED_A_PIN, self.LED_B_PIN, self.LED_X_PIN, self.LED_Y_PIN)

        led_a_pwm = GPIO.PWM(self.LED_A_PIN, 2000)
        led_a_pwm.start(0)

        led_b_pwm = GPIO.PWM(self.LED_B_PIN, 2000)
        led_b_pwm.start(0)

        led_x_pwm = GPIO.PWM(self.LED_X_PIN, 2000)
        led_x_pwm.start(0)

        led_y_pwm = GPIO.PWM(self.LED_Y_PIN, 2000)
        led_y_pwm.start(0)
        self.led_pwm_mapping = {self.LED_A_PIN: led_a_pwm,
                                self.LED_B_PIN: led_b_pwm,
                                self.LED_X_PIN: led_x_pwm,
                                self.LED_Y_PIN: led_y_pwm}

        # Setup motor driver
        GPIO.setup(self.MOTOR_EN_PIN, GPIO.OUT)
        GPIO.setup(self.MOTOR_LEFT_P, GPIO.OUT)
        GPIO.setup(self.MOTOR_LEFT_N, GPIO.OUT)
        GPIO.setup(self.MOTOR_RIGHT_P, GPIO.OUT)
        GPIO.setup(self.MOTOR_RIGHT_N, GPIO.OUT)

        motor_left_p_pwm = GPIO.PWM(self.MOTOR_LEFT_P, 100)
        motor_left_p_pwm.start(0)

        motor_left_n_pwm = GPIO.PWM(self.MOTOR_LEFT_N, 100)
        motor_left_n_pwm.start(0)

        motor_right_p_pwm = GPIO.PWM(self.MOTOR_RIGHT_P, 100)
        motor_right_p_pwm.start(0)

        motor_right_n_pwm = GPIO.PWM(self.MOTOR_RIGHT_N, 100)
        motor_right_n_pwm.start(0)
        self.motor_pwm_mapping = {self.MOTOR_LEFT_P: motor_left_p_pwm,
                                  self.MOTOR_LEFT_N: motor_left_n_pwm,
                                  self.MOTOR_RIGHT_P: motor_right_p_pwm,
                                  self.MOTOR_RIGHT_N: motor_right_n_pwm}

        try:
            self.sn3218 = sn3218.SN3218()
        except NameError:
            self.sn3218 = sn3218

        self.sn3218.reset()

        self.underlight = [0 for i in range(NUM_SN3218_CHANNELS)]
        self.sn3218.output(self.underlight)
        self.sn3218.enable_leds(0b111111111111111111)
        self.sn3218.disable()

        # setup ultrasonic sensor pins
        GPIO.setup(self.ULTRA_TRIG_PIN, GPIO.OUT)
        GPIO.setup(self.ULTRA_ECHO_PIN, GPIO.IN)

    def __del__(self):
        """ Clean up GPIO and underlighting when the class is deleted.
        """
        self.sn3218.disable()
        GPIO.cleanup()

    ############################################################
    ############################ Motors ########################
    ############################################################
    def set_motor_speed(self, motor, speed):
        """ Sets the speed of the given motor.

        Args:
        @param motor: the ID of the motor to set the state of
        @param speed: the motor speed, between -1.0 and 1.0
        """
        if type(motor) is not int:
            raise TypeError("motor must be an integer")

        if motor not in range(NUM_MOTORS):
            raise ValueError("""motor must be an integer in the range 0 to 1. For convenience, use the constants:
                MOTOR_LEFT (0), or MOTOR_RIGHT (1)""")

        # Limit the speed value rather than throw a value exception
        speed = max(min(speed, 1.0), -1.0)

        GPIO.output(self.MOTOR_EN_PIN, True)
        pwm_p = None
        pwm_n = None
        if motor == 0:
            # Left motor inverted so a positive speed drives forward
            pwm_p = self.motor_pwm_mapping[self.MOTOR_LEFT_N]
            pwm_n = self.motor_pwm_mapping[self.MOTOR_LEFT_P]
        else:
            pwm_p = self.motor_pwm_mapping[self.MOTOR_RIGHT_P]
            pwm_n = self.motor_pwm_mapping[self.MOTOR_RIGHT_N]

        if speed > 0.0:
            pwm_p.ChangeDutyCycle(100)
            pwm_n.ChangeDutyCycle(100 - (speed * 100))
        elif speed < 0.0:
            pwm_p.ChangeDutyCycle(100 - (-speed * 100))
            pwm_n.ChangeDutyCycle(100)
        else:
            pwm_p.ChangeDutyCycle(100)
            pwm_n.ChangeDutyCycle(100)

    def set_motor_speeds(self, l_speed, r_speed):
        """ Sets the speeds of both motors at once.

        Args:
        @param l_speed: the left motor speed, between -1.0 and 1.0
        @param r_speed: the right motor speed, between -1.0 and 1.0
        """
        self.set_motor_speed(MOTOR_LEFT, l_speed)
        self.set_motor_speed(MOTOR_RIGHT, r_speed)

    def set_left_speed(self, speed):
        """ Sets the speed of the left motor only.

        Args:
        @param speed: the speed, between -1.0 and 1.0
        """
        self.set_motor_speed(MOTOR_LEFT, speed)

    def set_right_speed(self, speed):
        """ Sets the speed of the right motor only.

        Args:
        @param speed: the speed, between -1.0 and 1.0
        """
        self.set_motor_speed(MOTOR_RIGHT, speed)

    def disable_motors(self):
        """ Disables both motors, allowing them to spin freely.
        """
        GPIO.output(self.MOTOR_EN_PIN, False)
        self.motor_pwm_mapping[self.MOTOR_LEFT_P].ChangeDutyCycle(0)
        self.motor_pwm_mapping[self.MOTOR_LEFT_N].ChangeDutyCycle(0)
        self.motor_pwm_mapping[self.MOTOR_RIGHT_P].ChangeDutyCycle(0)
        self.motor_pwm_mapping[self.MOTOR_RIGHT_N].ChangeDutyCycle(0)

    ###########################################################
    ##################### Motor Helpers #######################
    ###########################################################
    def forward(self, speed=1.0):
        """ Drives Trilobot forward.

        Args:
        @param speed: the speed to drive at, between 0.0 and 1.0
        """
        self.set_motor_speeds(speed, speed)

    def backward(self, speed=1.0):
        """ Drives Trilobot backward.

        Args:
        @params speed: the speed to drive at, between 0.0 and 1.0
        """
        self.set_motor_speeds(-speed, -speed)

    def turn_left(self, speed=1.0):
        """ Turns Trilobot left.

        Args:
        @params speed: the speed to turn at, between 0.0 and 1.0
        """
        self.set_motor_speeds(-speed, speed)

    def turn_right(self, speed=1.0):
        """ Turns Trilobot right.

        Args:
        @param speed: the speed to turn at, between 0.0 and 1.0
        """
        self.set_motor_speeds(speed, -speed)

    def curve_forward_left(self, speed=1.0):
        """ Drives Trilobot forward and to the left.

        Args:
        @param speed: the speed to drive at, between 0.0 and 1.0
        """
        self.set_motor_speeds(0.0, speed)

    def curve_forward_right(self, speed=1.0):
        """ Drives Trilobot forward and to the right.

        Args:
        @param speed: the speed to drive at, between 0.0 and 1.0
        """
        self.set_motor_speeds(speed, 0.0)

    def curve_backward_left(self, speed=1.0):
        """ Drives Trilobot backward and to the left.

        Args:
        @param speed: the speed to drive at, between 0.0 and 1.0
        """
        self.set_motor_speeds(0.0, -speed)

    def curve_backward_right(self, speed=1.0):
        """ Drives Trilobot backward and to the right.

        Args:
        @param speed: the speed to drive at, between 0.0 and 1.0
        """
        self.set_motor_speeds(-speed, 0.0)

    def stop(self):
        """ Stops Trilobot from driving, sharply.
        """
        self.set_motor_speeds(0.0, 0.0)

    def coast(self):
        """ Stops Trilobot from driving, slowly.
        """
        self.disable_motors()

    ###########################################################
    ####################### Underlights #######################
    ###########################################################
    def show_underlighting(self):
        """ Shows the previously stored colors on Trilobot's underlights.
        """
        self.sn3218.output(self.underlight)
        self.sn3218.enable()

    def disable_underlighting(self):
        """ Disables Trilobot's underlighting, preserving the last set colors.
        """
        self.sn3218.disable()

    def set_underlight(self, light: int, color: Union[list, tuple], show=True):
        """ Sets a single underlight to a given RGB color.

        Args:
        @param light: the ID of the light to set the color of
        @param color: list/tuple containing all three color components (from 0 to 255).
        @param show: whether or not to show the new color immediately
        """
        if type(light) is not int:
            raise TypeError("light must be an integer")

        if light not in range(NUM_UNDERLIGHTS):
            raise ValueError("""light must be an integer in the range 0 to 5. For convenience, use the constants:
                LIGHT_FRONT_RIGHT (0), LIGHT_FRONT_LEFT (1), LIGHT_MIDDLE_LEFT (2), LIGHT_REAR_LEFT (3), LIGHT_REAR_RIGHT (4), or LIGHT_MIDDLE_RIGHT (5)""")

        if isinstance(color, list) or isinstance(color, tuple):
            self.underlight[(light * 3)] = int(color[0])
            self.underlight[(light * 3) + 1] = int(color[1])
            self.underlight[(light * 3) + 2] = int(color[2])
        else:
            raise ValueError("color must either be a color hex code, or a list/tuple of 3 numbers between 0 and 255")

        if show:
            self.show_underlighting()

    def fill_underlighting(self, color: Union[list, tuple], show=True):
        """ Fill all the underlights with a given RGB color.

        Args:
        @param color: list/tuple containing all three color components (from 0 to 255).
        @param show: whether or not to show the new color immediately
        """
        for i in range(0, NUM_UNDERLIGHTS):
            self.set_underlight(i, color, show=False)
        if show:
            self.show_underlighting()

    def clear_underlight(self, light: int, show=True):
        """ Clear the color of a single underlight. This has the effect of turning it off.

        Args:
        @param light: the ID of the light to clear
        @param show: whether or not to show the new color immediately
        """
        self.set_underlight(light, (0, 0, 0), show=show)

    def clear_underlighting(self, show=True):
        """ Clear the color of all underlights. This has the effect of turning them off.

        Args:
        @param show: whether or not to show the new color immediately
        """
        self.fill_underlighting((0, 0, 0), show=show)

    ###########################################################
    ################### Underlight Helpers ####################
    ###########################################################
    def set_underlights(self, lights: Union[list, tuple], color: Union[list, tuple], show=True):
        """ Sets a group of underlights to a given RGB color.

        Args: 
        @param lights: a list/tuple containing the IDs of the lights to set the color of
        @param color: a list/tuple containing all three color components.
        @param show: whether or not to show the new color immediately
        """
        if type(lights) != list and type(lights) != tuple:
            raise TypeError("lights must be a list or tuple containing the numbers for the underlights to set (from 0 to 5)")

        light_count = len(lights)
        if light_count > NUM_UNDERLIGHTS:
            raise ValueError("lights contains more values than the number of underlights available")
        if light_count == 0:
            raise ValueError("lights cannot be empty")

        if light_count > 1:
            for i in range(0, light_count - 1):
                self.set_underlight(lights[i], color, show=False)

        self.set_underlight(lights[light_count - 1], color, show=show)

    def clear_underlights(self, lights: Union[list, tuple], show=True):
        """ Clear the color of a group of underlights. This has the effect of turning them off.

        Args:
        @param lights: a list/tuple containing the IDs of the lights to set the color of
        @param show: whether or not to show the new color immediately
        """
        self.set_underlights(lights, (0, 0, 0), show=show)
        
    ###########################################################
    ################### Ultrasonic Sensor #####################
    ###########################################################
    def read_distance(self):
        """ Reade distance in cm with ultrasonic sensor
        
        Returns:
        @param distance: detected distance
        """
        GPIO.output(self.ULTRA_TRIG_PIN, 1)
        time.sleep(0.00001)
        GPIO.output(self.ULTRA_TRIG_PIN, 0)
    
        StartTime = time.time()
        StopTime = time.time()
    
        # # save StartTime
        # while GPIO.input(self.ULTRA_ECHO_PIN) == 0:
        #     StartTime = time.time()
    
        # # save time of arrival
        # while GPIO.input(self.ULTRA_ECHO_PIN) == 1:
        #     StopTime = time.time()
    
        # time difference between start and arrival
        TimeElapsed = StopTime - StartTime
        # multiply with the sonic speed (34300 cm/s)
        # and divide by 2, because there and back
        distance = (TimeElapsed * 34300) / 2        
        return distance 