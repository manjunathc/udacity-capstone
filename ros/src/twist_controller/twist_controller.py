import rospy
from yaw_controller import YawController
from pid import PID
GAS_DENSITY = 2.858
ONE_MPH = 0.44704
import time;

class Controller(object):
    def __init__(self, *args, **kwargs):
        # TODO: Implement
	# testing initialization is not giving error
	self.stopWatch = time.time()*1000
	#self.Lowpass = LowPassFilter(0.3,1)
	self.PIDThrottle = PID(2.9226855,10.3267511,0.49327083)
	self.PIDBrake = PID(2.9226855,10.3267511,0.49327083)
	self.YawCtrl = YawController(args[0], args[1], args[2], args[3], args[4])
        pass

    def control(self, *args, **kwargs):
	rospy.loginfo('%s %s %s', args[0], args[1], args[2])
        steer = self.YawCtrl.get_steering(args[0], args[1], args[2])
	accel = PIDThrottle.step(args[0]-args[2], time.time()*1000-self.stopWatch)
	brake = PIDBrake.step(args[0]-args[2], time.time()*1000-self.stopWatch)
	self.stopWatch = time.time()*1000
        # Return throttle, brake, steer
        return accel, brake, steer
