from yaw_controller import YawController
from pid import PID
from lowpass import LowPassFilter
from rospy import logwarn, get_time

from math import atan2, sin, cos

GAS_DENSITY = 2.858
ONE_MPH = 0.44704

SLOPE = (0.215 - 0.11)/(11.11 - 0.)
BIAS = 0.11
#Got this value from the slack channel
#Will remove hardcoded number at a later date if it is useless and replace with the parameter from ROS
class Controller(object):
	def __init__(self, wheel_base, steer_ratio, min_speed, max_lat_accel, max_steer_angle, decel_limit, accel_limit, brake_deadband):
		self.max_angle = max_steer_angle
		self.previous_time = get_time()
		self.decel_limit = decel_limit
		self.accel_limit = accel_limit
		self.brake_deadband = brake_deadband
		self.previous_velocity = None

		#Using the Udacity provided controller for steering. It doesn't need to to be reset after initialization
		self.yaw_controller = YawController(wheel_base, steer_ratio, min_speed, max_lat_accel, max_steer_angle)
		self.set_controllers()

	def control(self, linear_velocity, angular_velocity, current_linear, dbw_enabled):
	#Reset if drive by wire is not enabled and return 0's		
		if not dbw_enabled:
			self.set_controllers()
			return 0., 0., 0.
	
		throttle, brake, steer = 0., 0., 0.
	#Calculate CTE for throttle
		time = get_time()
		time_elapsed, self.previous_time = time - self.previous_time, time
		dv = linear_velocity - current_linear
		if dv > 0.:
			#throttle = self.pid_throttle.step(dv, time_elapsed) if current_linear > 5. else self.pid_throttle_low.step(dv, time_elapsed)
			velocity = min(linear_velocity, current_linear + 0.9*self.accel_limit*time_elapsed)
			cte = self.previous_velocity - current_linear if not self.previous_velocity is None else 0.
			throttle = SLOPE*velocity + BIAS + self.pid_throttle.step(cte, time_elapsed)
			throttle = max(throttle, 0.)
		else:
			self.previous_velocity = None
			velocity = max(0., min(linear_velocity, current_linear + 0.9*self.decel_limit*time_elapsed))
			dv = current_linear-velocity
			brake = self.pid_brake.step(dv, time_elapsed) if dv > self.brake_deadband*time_elapsed else 0.

	#Steer uses provided yaw controller
		steer = self.yaw_controller.get_steering(linear_velocity, angular_velocity, current_linear)

	#Apply low pass filter to smooth out steering
		steer = self.lowpass_steer.filt(steer)
	# Return throttle, brake, steer
		return throttle, brake, steer

	def set_controllers(self):
		#PID Controllers for throttle and brake
		#self.pid_throttle_low = PID(0.15,0.0,0.0,0.01,0.12)
		#self.pid_throttle = PID(0.4,0.0,0.0,0.12,0.25)
		self.pid_brake = PID(0.15,0.0,0.0,0.0,0.22)
		#Low pass filter to smooth out the steering
		self.pid_throttle = PID(0.1,0.0,0.0,-0.1,0.1)
		self.lowpass_steer = LowPassFilter(0.2,1.0)
