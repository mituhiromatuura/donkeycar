"""
actuators.py
Classes to control the motors and servos. These classes
are wrapped in a mixer class before being used in the drive loop.
"""

import time
import donkeycar as dk
import RPi.GPIO as GPIO

class PWMSteering:
    """
    Wrapper over a PWM motor cotnroller to convert angles to PWM pulses.
    """
    LEFT_ANGLE = -1
    RIGHT_ANGLE = 1

    def __init__(self, controller=None,
                       left_pulse=290,
                       right_pulse=490,
                       dir=1):

        self.controller = controller
        self.left_pulse = left_pulse
        self.right_pulse = right_pulse
        self.dir = dir


    def run(self, angle):
        angle *= self.dir
        #map absolute angle to angle that vehicle can implement.
        pulse = dk.util.data.map_range(angle,
                                        self.LEFT_ANGLE, self.RIGHT_ANGLE,
                                        self.left_pulse, self.right_pulse)

        self.controller.set_pulse(pulse)

    def shutdown(self):
        self.run(0) #set steering straight



class PWMThrottle:
    """
    Wrapper over a PWM motor cotnroller to convert -1 to 1 throttle
    values to PWM pulses.
    """
    MIN_THROTTLE = -1
    MAX_THROTTLE =  1

    def __init__(self, controller_l=None,
                       controller_r=None,
                       max_pulse=300,
                       min_pulse=490,
                       zero_pulse=350,
                       dir=1):

        self.controller_l = controller_l
        self.controller_r = controller_r
        self.max_pulse = max_pulse
        self.min_pulse = min_pulse
        self.zero_pulse = zero_pulse
        self.dir = dir

        #send zero pulse to calibrate ESC
        self.controller_l.set_pulse(self.zero_pulse)
        self.controller_r.set_pulse(self.zero_pulse)

        # ST L298N
        GPIO.setmode(GPIO.BOARD)
        # IN1
        GPIO.setup(11, GPIO.OUT)
        GPIO.output(11, 0)
        # IN2
        GPIO.setup(12, GPIO.OUT)
        GPIO.output(12, 0)
        # IN3
        GPIO.setup(13, GPIO.OUT)
        GPIO.output(13, 0)
        # IN4
        GPIO.setup(15, GPIO.OUT)
        GPIO.output(15, 0)

        time.sleep(1)


    def run(self, throttle):
        throttle *= self.dir
        if throttle > 0:
            pulse = dk.util.data.map_range(throttle,
                                                    0, self.MAX_THROTTLE,
                                                    self.zero_pulse, self.max_pulse)
            GPIO.output(11, 1)
            GPIO.output(12, 0)
            GPIO.output(13, 0)
            GPIO.output(15, 1)

        else:
            pulse = dk.util.data.map_range(throttle,
                                                    self.MIN_THROTTLE, 0,
                                                    self.min_pulse, self.zero_pulse)
            GPIO.output(11, 0)
            GPIO.output(12, 1)
            GPIO.output(13, 1)
            GPIO.output(15, 0)

        self.controller_l.set_pulse(pulse)
        self.controller_r.set_pulse(pulse)

    def shutdown(self):
        self.run(0) #stop vehicle
