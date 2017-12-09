# modes:
# 	vertical speed hold		(for pitch)
#	altitude hold			(for pitch)
# 	attitude hold			(pitch, roll, yaw)
#	heading hold			(roll, yaw)

from PID import PID
from attitude_control import attitude_control
from autothrottle import autothrottle
from flight_telemetry import telemetry_stream
import numpy as np

# TODO: targeting both vertical speed and altitude
# TODO: altitude should go through vertical speed targeting
# BUG: altitude pitch target too drastic
class autopilot:
	def __init__(self, vessel, conn, axis_scaling=(1, 1, 1)):
		self.heading_roll_target_PID = PID(P=5, I=0.5, D=1, lower=-40, upper=40)
		self.heading_roll_target_PID.error_clamp_180 = True
		self.heading_roll_target_PID.windup_guard = 2	# 2 * 0.5 = 1 degree roll
		self.altitude_pitch_target_PID = PID(P=0.03, I=0.006, D=0.003, lower=-45, upper=45)
		self.altitude_pitch_target_PID.windup_guard = 500 # 500 * 0.006 = 3 degrees pitch
		self.vs_pitch_target_PID = PID(P=1, I=0.1, D=0.1, lower=-45, upper=45)
		self.attitude_pilot = attitude_control(vessel, conn, axis_scaling)
		self.at_pilot = autothrottle(vessel, conn)

		self.conn = conn
		self.vessel = vessel
		self.telemetry = self.attitude_pilot.telemetry
		self.reference_frame = vessel.orbit.body.reference_frame
		self.surf_vessel_reference_frame = conn.space_center.ReferenceFrame.create_hybrid(
											    position=vessel.orbit.body.reference_frame,
											    rotation=vessel.surface_reference_frame,
										   )
		self.vessel_altitude_reading = self.telemetry.vessel_mean_altitude
		self.hold_target = [('off', 0.0), ('off', 0.0), ('raw', 0.0), ('off', 0.0)]	# pitch, roll, yaw, throttle
		self.attitude_target = [0.0, 0.0, 0.0]

		# initialize to hold current attitude
		for i in range(len(self.attitude_target)):
			self.hold_target[i] = ('raw', self.telemetry.vessel_attitude[i]())
			self.attitude_target[i] = self.telemetry.vessel_attitude[i]()

	def set_altitude_type(self, altitude_type):
		if altitude_type == 'mean':
			self.vessel_altitude_reading = self.telemetry.vessel_mean_altitude
			self.altitude_pitch_target_PID.clear()
		elif altitude_type == 'surface':
			self.vessel_altitude_reading = self.telemetry.vessel_surface_altitude
			self.altitude_pitch_target_PID.clear()
		else:
			raise ValueError("Invalid altitude_type. Supported: 'mean' or 'surface'")

	def set_target(self, vs=None, altitude=None, heading=None, pitch=None, roll=None, sideslip=None, speed=None, throttle=None):
		# pitch
		if vs is not None:
			self.hold_target[0] = ('vs', vs)
			self.vs_pitch_target_PID.clear()
			self.vs_pitch_target_PID.setpoint = vs / self.telemetry.vessel_speed()
		if altitude is not None:
			if vs is not None:
				raise Warning('Altitude target overrides vertical speed target.') 
			self.hold_target[0] = ('altitude', altitude)
			self.altitude_pitch_target_PID.clear()
			self.altitude_pitch_target_PID.setpoint = altitude
		if pitch is not None:
			if vs is not None:
				raise Warning('Raw pitch target overrides vertical speed target.')
			if altitude is not None:
				raise Warning('Raw pitch target overrides altitude target.')
			if pitch == 'off':
				self.hold_target[0] = ('off', 0.0)
			elif pitch == 'current':
				self.hold_target[0] = ('raw', self.telemetry.vessel_attitude[0]())
			else:
				self.hold_target[0] = ('raw', pitch)

		# roll
		if heading is not None:
			self.hold_target[1] = ('heading', heading)
			self.heading_roll_target_PID.setpoint = heading
		if roll is not None:
			if heading is not None:
				raise Warning('Raw roll target overrides heading target.')
			self.hold_target[1] = ('raw', roll)
			if roll == 'off':
				self.hold_target[1] = ('off', 0.0)
			elif roll == 'current':
				self.hold_target[1] = ('raw', self.telemetry.vessel_attitude[1]())
			else:
				self.hold_target[1] = ('raw', roll)

		# yaw
		if sideslip is not None:
			if sideslip == 'off':
				self.hold_target[2] = ('off', 0.0)
			elif sideslip == 'current':
				self.hold_target[2] = ('raw', self.telemetry.vessel_attitude[2]())
			else:
				self.hold_target[2] = ('raw', sideslip)

		# autothrottle
		if speed is not None:
			self.hold_target[3] = ('speed', speed)
			self.at_pilot.set_target(speed)
		if throttle is not None:
			if speed is not None:
				raise Warning('Raw throttle target overrides speed target.')
			self.hold_target[3] = ('raw', throttle)

	def update(self):
		# pitch
		if self.hold_target[0][0] == 'vs':
			vs = self.telemetry.vessel_vs()
			speed = self.telemetry.vessel_speed()
			current_surface_v_pitch = np.rad2deg(np.arcsin(vs / speed))
			target_surface_v_pitch = np.rad2deg(np.arcsin(self.hold_target[0][1] / speed))
			vessel_sideslip = self.telemetry.vessel_attitude[2]()
			self.vs_pitch_target_PID.setpoint = target_surface_v_pitch
			self.vs_pitch_target_PID.update(current_surface_v_pitch, timestamp=self.telemetry.vessel_met())
			# [pitch target] = [surface velocity pitch] + [aoa * sin(roll)] + [sideslip * sin(roll)] + [closed loop error term]
			self.attitude_target[0] = current_surface_v_pitch + (self.telemetry.vessel_aoa() + vessel_sideslip) * np.sin(np.deg2rad(self.telemetry.vessel_attitude[1]())) + self.vs_pitch_target_PID.output
		elif self.hold_target[0][0] == 'altitude':
			self.altitude_pitch_target_PID.update(self.vessel_altitude_reading(), timestamp=self.telemetry.vessel_met())
			self.attitude_target[0] = self.altitude_pitch_target_PID.output
		elif self.hold_target[0][0] == 'raw':
			self.attitude_target[0] = self.hold_target[0][1]

		# roll
		if self.hold_target[1][0] == 'heading':
			self.heading_roll_target_PID.update(self.telemetry.vessel_surface_v_heading(), timestamp=self.telemetry.vessel_met())
			self.attitude_target[1] = self.heading_roll_target_PID.output
		elif self.hold_target[1][0] == 'raw':
			self.attitude_target[1] = self.hold_target[1][1]

		# yaw
		if self.hold_target[2][0] == 'raw':
			self.attitude_target[2] = self.hold_target[2][1]

		self.attitude_pilot.set_target(self.attitude_target)
		self.attitude_pilot.update()

		# autothrottle
		if self.hold_target[3][0] == 'speed':
			self.at_pilot.update()

	def steer(self):
		self.attitude_pilot.steer()
		if self.hold_target[3][0] == 'speed':
			self.at_pilot.steer()
		elif self.hold_target[3][0] == 'raw':
			self.vessel.control.throttle = self.hold_target[3][1]

if __name__ == '__main__':
	import krpc
	import os
	conn = krpc.connect(name='Attitude', address='127.0.0.1')
	vessel = conn.space_center.active_vessel
	ap = autopilot(vessel, conn, axis_scaling = (0.25, 0.05, 0.5))
	ap.set_target(altitude=3000, heading=260, speed=200.0)

	while True:
		ap.update()
		ap.steer()
		os.system('clear')
		# for i in range(len(attitude_pilot.angular_velocity_PIDs)):
		# 	print '%.3f\t%.3f\t%.3f\t%.3f' % (attitude_pilot.angular_velocity_PIDs[i].PTerm, attitude_pilot.angular_velocity_PIDs[i].ITerm, attitude_pilot.angular_velocity_PIDs[i].DTerm, attitude_pilot.angular_velocity_PIDs[i].output)
		# print
		# for elem in attitude_pilot.max_angular_velocities:
		# 	print '%.3f\t' % elem,
		# print attitude_pilot.gain_schedule_scale
