import rospy
from yaw_controller import YawController
from pid import PID
import time;
GAS_DENSITY = 2.858
ONE_MPH = 0.44704

class Controller(object):
    def __init__(self, *args, **kwargs):
        # TODO: Implement
	# testing initialization is not giving error
	self.stopWatch = time.time()*1000
	#self.Lowpass = LowPassFilter(0.3,1)
	self.PIDThrottle = PID(0.8,0,0.05)
	self.YawCtrl = YawController(args[0], args[1], args[2], args[3], args[4])

    def control(self, linear_velocity, angular_velocity, current_velocity):
        steer = self.YawCtrl.get_steering(linear_velocity, angular_velocity, current_velocity)

        time_delta = time.time()*1000 - self.stopWatch
	self.stopWatch += time_delta
        accel = self.PIDThrottle.step(linear_velocity-current_velocity, time_delta)

        if accel > 0:
            throttle = accel
            breaking = 0.
        else:
            breaking = -accel
            throttle = 0.
       
        # Return throttle, brake, steer
        return throttle, breaking, steer

