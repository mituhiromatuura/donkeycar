"""
actuators.py
Classes to control the motors and servos. These classes
are wrapped in a mixer class before being used in the drive loop.
"""

import time
import donkeycar as dk

g_angle = 0

class PimouseSteering:

    def __init__(self):

        global g_angle
        g_angle = 0


    def run(self, angle):

        global g_angle
        g_angle = angle

    def shutdown(self):
        self.run(0) #set steering straight



class PimouseThrottle:

    MIN_THROTTLE = -1
    MAX_THROTTLE =  1

    def __init__(self, controller=None,
                       max_pulse=0,
                       min_pulse=0,
                       zero_pulse=0):

        self.max_pulse = max_pulse
        self.min_pulse = min_pulse
        self.zero_pulse = zero_pulse

        #send zero pulse to calibrate ESC
        self.run(0)

        try:
            with open("/dev/rtmotoren0",'w') as f:
                f.write("1\n")
        except:
            print("/dev/rtmotoren0 open error")

        time.sleep(1)


    def run(self, throttle):
        global g_angle

        if throttle > 0:
            pulse_l = dk.util.data.map_range(throttle * (1.0 + g_angle),
                                                    0, self.MAX_THROTTLE,
                                                    self.zero_pulse, self.max_pulse)
            pulse_r = dk.util.data.map_range(throttle * (1.0 - g_angle),
                                                    0, self.MAX_THROTTLE,
                                                    self.zero_pulse, self.max_pulse)

        else:
            pulse_l = dk.util.data.map_range(throttle * (1.0 + g_angle),
                                                    self.MIN_THROTTLE, 0,
                                                    self.min_pulse, self.zero_pulse)
            pulse_r = dk.util.data.map_range(throttle * (1.0 - g_angle),
                                                    self.MIN_THROTTLE, 0,
                                                    self.min_pulse, self.zero_pulse)

        try:
            with open("/dev/rtmotor_raw_l0",'w') as lf,\
                 open("/dev/rtmotor_raw_r0",'w') as rf:
                lf.write(str(int(round(pulse_l))) + "\n")
                rf.write(str(int(round(pulse_r))) + "\n")
        except:
            print("/dev/rtmotor_raw_x0 open error")

    def shutdown(self):
        self.run(0) #stop vehicle
