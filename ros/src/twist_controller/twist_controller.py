from pid import PID
from yaw_controller import YawController
from lowpass import LowPassFilter
GAS_DENSITY = 2.858
ONE_MPH = 0.44704


class Controller(object):
    def __init__(self, *args, **kwargs):
        # TODO: Implement
	# testing initialization is not giving error
	self.Lowpass = LowPassFilter(0.3,1)
	self.PID = PID(1,1,1)
	self.YawCtrl = YawController(args[0], args[1], args[2], args[3], args[4])
        pass

    def control(self, *args, **kwargs):
        # TODO: Change the arg, kwarg list to suit your needs
        # Return throttle, brake, steer
        return 1., 0., 0.
