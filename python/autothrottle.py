from PID import PID
from flight_telemetry import telemetry_stream

class autothrottle:
	def __init__(self, vessel, conn):
		self.thrust_PID = PID(P=0.05, I=0.01, D=0.03, lower=0, upper=1)
		self.thrust_PID.windup_guard = 10.0

		self.throttle_PID = PID(P=1, I=0.1, D=0.2, lower=-1, upper=1)
		self.throttle_PID.setpoint = self.thrust_PID.output
		self.throttle_PID.windup_guard = 1.0

		self.throttle = 0.0
		self.reference_frame = vessel.orbit.body.reference_frame

		self.vessel = vessel
		self.telemetry = telemetry_stream(vessel, conn)

	def set_target(self, target):
		self.thrust_PID.setpoint = target

	def update(self):
		if not self.telemetry.time_elapsed():
			return
		if self.telemetry.vessel_max_thrust()() == 0:
			raise ValueError('vessel has no usable thrust. Autothrottle will not update.')
		else:
			self.thrust_PID.update(self.telemetry.vessel_speed(), timestamp=self.telemetry.vessel_met())
			self.throttle_PID.setpoint = self.thrust_PID.output
			self.throttle_PID.update(self.telemetry.vessel_thrust() / self.telemetry.vessel_max_thrust()(), timestamp=self.telemetry.vessel_met())
			if self.throttle_PID.output >= 0:
				self.throttle = self.thrust_PID.output + self.throttle_PID.output * (1 - self.thrust_PID.output)
			else:
				self.throttle = self.thrust_PID.output * (1 - self.throttle_PID.output)

	def steer(self):
		self.vessel.control.throttle = self.throttle

if __name__ == '__main__':
	import krpc
	speed_target = 200
	conn = krpc.connect(name='Autothrottle', address='127.0.0.1')
	vessel = conn.space_center.active_vessel
	at = autothrottle(vessel, conn)
	at.set_target(speed_target)

	while True:
		at.update()
		at.steer()
