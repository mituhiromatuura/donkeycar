import donkeycar as dk

class PWMPanTilt:
    """
    Wrapper over a PWM motor cotnroller to convert angles to PWM pulses.
    """
    LEFT_ANGLE = -1
    RIGHT_ANGLE = 1

    def __init__(self, controller=None,
                       left_pulse=140,
                       right_pulse=640,
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
        self.run(0) #set straight
