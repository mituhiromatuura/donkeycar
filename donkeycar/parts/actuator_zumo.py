"""
actuators.py
Classes to control the motors and servos. These classes
are wrapped in a mixer class before being used in the drive loop.
"""

import time
import donkeycar as dk
import RPi.GPIO as GPIO

class PCA9685:
    """
    PWM motor controler using PCA9685 boards.
    This is used for most RC Cars
    """
    def __init__(self, channel, frequency=60):
        import Adafruit_PCA9685
        # Initialise the PCA9685 using the default address (0x40).
        self.pwm = Adafruit_PCA9685.PCA9685()
        self.pwm.set_pwm_freq(frequency)
        self.channel = channel

    def set_pulse(self, pulse):
        self.pwm.set_pwm(self.channel, 0, pulse)

    def run(self, pulse):
        self.set_pulse(pulse)

g_angle = 0
g_controller_l = PCA9685(0, 12000)
g_controller_r = PCA9685(1, 12000)

class ZumoSteering:
    """
    Wrapper over a PWM motor cotnroller to convert angles to PWM pulses.
    """
    LEFT_ANGLE = -1
    RIGHT_ANGLE = 1

    def __init__(self, controller=None,
                       left_pulse=290,
                       right_pulse=490):

        self.left_pulse = left_pulse
        self.right_pulse = right_pulse


    def run(self, angle):
        global g_angle
        g_angle = angle

    def shutdown(self):
        self.run(0) #set steering straight



class ZumoThrottle:
    """
    Wrapper over a PWM motor cotnroller to convert -1 to 1 throttle
    values to PWM pulses.
    """
    MIN_THROTTLE = -1
    MAX_THROTTLE =  1

    def __init__(self, controller=None,
                       max_pulse=300,
                       min_pulse=490,
                       zero_pulse=350):

        self.max_pulse = max_pulse
        self.min_pulse = min_pulse
        self.zero_pulse = zero_pulse

        #send zero pulse to calibrate ESC
        g_controller_l.set_pulse(self.zero_pulse)
        g_controller_r.set_pulse(self.zero_pulse)

        GPIO.setmode(GPIO.BOARD)
        GPIO.setup(11, GPIO.OUT)
        GPIO.output(11, 0)
        GPIO.setup(12, GPIO.OUT)
        GPIO.output(12, 0)

        time.sleep(1)


    def run(self, throttle):
        global g_angle
        global g_controller_l
        global g_controller_r

        if throttle > 0:
            pulse_l = dk.util.data.map_range(throttle * (1.0 - g_angle),
                                                    0, self.MAX_THROTTLE,
                                                    self.zero_pulse, self.max_pulse)
            pulse_r = dk.util.data.map_range(throttle * (1.0 + g_angle),
                                                    0, self.MAX_THROTTLE,
                                                    self.zero_pulse, self.max_pulse)
            GPIO.output(11, 0)
            GPIO.output(12, 0)
        else:
            pulse_l = dk.util.data.map_range(throttle * (1.0 - g_angle),
                                                    self.MIN_THROTTLE, 0,
                                                    self.max_pulse, self.zero_pulse)
            pulse_r = dk.util.data.map_range(throttle * (1.0 + g_angle),
                                                    self.MIN_THROTTLE, 0,
                                                    self.max_pulse, self.zero_pulse)
            GPIO.output(11, 1)
            GPIO.output(12, 1)

        g_controller_l.set_pulse(pulse_l)
        g_controller_r.set_pulse(pulse_r)

    def shutdown(self):
        self.run(0) #stop vehicle
