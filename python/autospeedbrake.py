from PID import PID

class autospeedbrake:
	def __init__(self, target):
		self.drag_PID = PID(P=0.05, I=0.01, D=0.03, lower=0, upper=1)
		self.drag_PID.windup_guard = 10.0
		self.drag.setpoint = target

		self.speedbrake_PID = PID(P=10, I=2, D=1, lower=-1, upper=1)
		self.speedbrake_PID.setpoint = self.drag_PID.output
		
		self.speedbrake = 0.0

	def update(self, speed):

		# calculate thrust error
		# update thrust PID
		# update throttle PID setpoint
		# calculate throttle error
		# update throttle PID
		# update throttle
		if self.speedbrake_PID.output >= 0:
			self.throttle = self.thrust_PID.output + self.speedbrake_PID.output * (1 - self.thrust_PID.output)
		else:
			self.throttle = self.thrust_PID.output + self.speedbrake_PID.output * self.thrust_PID.output

	def steer(self, vehicle):
		# steer vehicle
	