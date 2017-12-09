from PID import PID
import utilities
import numpy as np
from flight_telemetry import telemetry_stream

class attitude_control:
	def __init__(self, vessel, conn, axis_scaling=(1, 1, 1)):	# TODO: auto-tune axis scaling with reference torque/MOI
		# error in pitch --> requested angvel --> error in angvel --> requested torque --> control
		# TODO: auto-tune angvel PIDs with available torque, moment of inertia, and deceleration_time
		# TODO: auto-tune control PIDs with Q; check if that is redundant with torque/MoI tuning
		# lists are ordered in pitch, roll, yaw
		self.max_angular_velocities = [5.0, 15.0, 5.0]
		self.angular_velocity_PID_params = [
			{'P':0.5, 'I':0.025, 'D':0.025, 'lower':-5, 'upper':5},
			{'P':0.5, 'I':0.05, 'D':0.025, 'lower':-15, 'upper':15},
			{'P':-0.5, 'I':-0.05, 'D':-0.025, 'lower':-5, 'upper':5},
		]
		self.angular_velocity_PID_windup_guards = [1, 1, 1]

		self.angular_velocity_PIDs = []
		for elem in self.angular_velocity_PID_params:
			self.angular_velocity_PIDs.append(PID(**elem))

		for i in range(len(self.angular_velocity_PID_windup_guards)):
			self.angular_velocity_PIDs[i].windup_guard = self.angular_velocity_PID_windup_guards[i]

		self.control_PID_params = [
			{'P': 0.40, 'I': 0.10, 'D': 0.08, 'lower': -1, 'upper': 1},
			{'P': 0.20, 'I': 0.005, 'D': 0.04, 'lower': -1, 'upper': 1},
			{'P': 0.50, 'I': 0.05, 'D': 0.20, 'lower': -1, 'upper': 1},
		]

		for i in range(len(self.control_PID_params)):
			for term in ['P', 'I', 'D']:
				self.control_PID_params[i][term] *= axis_scaling[i]

		self.control_PIDs = []
		for elem in self.control_PID_params:
			self.control_PIDs.append(PID(**elem))

		self.conn = conn
		self.vessel = vessel
		self.telemetry = telemetry_stream(vessel, conn)
		self.reference_frame = vessel.orbit.body.reference_frame
		self.set_autotune(torque=False, dynamic_pressure=True)

	def set_autotune(self, torque=True, dynamic_pressure=True, deceleration_time=1.0, Q_ref=11520.0, overshoot=(0.01, 0.01, 0.01), time_to_peak=(3.0, 3.0, 3.0)):
		self.autotune_torque = torque
		self.autotune_dynamic_pressure = dynamic_pressure
		self.autotune_deceleration_time = deceleration_time
		self.autotune_Q_ref = Q_ref
		self.autotune_overshoot = overshoot
		self.autotune_time_to_peak = time_to_peak

	def autotune(self):
		if self.autotune_torque:
			# constrain max angular velocity by available torque, moment of inertia and deceleration time
			for i in range(len(self.max_angular_velocities)):
				self.max_angular_velocities[i] = self.telemetry.vessel_available_torque()[0][i] / self.telemetry.vessel_moment_of_inertia()[i] * self.autotune_deceleration_time
				self.angular_velocity_PIDs[i].lower = -self.max_angular_velocities[i]
				self.angular_velocity_PIDs[i].upper = self.max_angular_velocities[i]
			# # autotune PIDs; translated from krpc autopilot 'DoAutoTuneAxis'
			# self._calc_autotune_parameters()
			# acceleration_inv = np.array(self.telemetry.vessel_moment_of_inertia()) / np.array(self.telemetry.vessel_available_torque()[0])
			# Kp = self.twice_zeta_omega * acceleration_inv
			# Ki = self.omega_squared * acceleration_inv
			# for i in range(len(self.angular_velocity_PIDs)):
			# 	if acceleration_inv[i] < 1000:
			# 		self.angular_velocity_PIDs[i].Kp = Kp[i]
			# 		self.angular_velocity_PIDs[i].Ki = Ki[i]

		if self.autotune_dynamic_pressure:
			# autotune control PIDs by current dynamic pressure
			self.gain_schedule_scale = self.autotune_Q_ref / float(self.telemetry.vessel_Q())
			for i in range(len(self.control_PIDs)):
				self.control_PIDs[i].Kp = self.control_PID_params[i]['P'] * self.gain_schedule_scale
				self.control_PIDs[i].Ki = self.control_PID_params[i]['I'] * self.gain_schedule_scale
				self.control_PIDs[i].Kd = self.control_PID_params[i]['D'] * self.gain_schedule_scale

	def _calc_autotune_parameters(self):
		# translated from krpc autopilot 'UpdatePIDParameters'
		self.twice_zeta_omega = np.zeros(len(self.angular_velocity_PIDs))
		self.omega_squared = np.zeros(len(self.angular_velocity_PIDs))
		sq_log_overshoot = np.log(self.autotune_overshoot) ** 2
		zeta = np.sqrt(sq_log_overshoot / (np.pi ** 2 + sq_log_overshoot))
		omega = np.pi / (np.array(self.autotune_time_to_peak) * np.sqrt(1.0 - zeta ** 2))
		self.twice_zeta_omega = 2 * zeta * omega
		self.omega_squared = omega ** 2

	# target: tuple (pitch, roll, yaw)
	def set_target(self, target):
		for i in range(len(target)):
			self.angular_velocity_PIDs[i].setpoint = target[i]

	def update(self):
		self.autotune()
		current_angvel = self.vessel.angular_velocity(self.reference_frame)
		for i in range(len(self.angular_velocity_PIDs)):
			self.angular_velocity_PIDs[i].update(self.telemetry.vessel_attitude[i](), timestamp=self.telemetry.vessel_met())
			self.control_PIDs[i].setpoint = self.angular_velocity_PIDs[i].output
			self.control_PIDs[i].update(current_angvel[i])

	def steer(self):
		self.vessel.control.pitch = self.control_PIDs[0].output
		self.vessel.control.roll = self.control_PIDs[1].output
		self.vessel.control.yaw = self.control_PIDs[2].output


if __name__ == '__main__':
	import krpc
	import os
	attitude_target = (7.0, 0.0, 0.0)
	conn = krpc.connect(name='Attitude', address='127.0.0.1')
	vessel = conn.space_center.active_vessel
	attitude_pilot = attitude_control(vessel, conn, axis_scaling = (0.25, 0.05, 0.5))
	attitude_pilot.set_target(attitude_target)

	while True:
		attitude_pilot.update()
		attitude_pilot.steer()
		os.system('clear')
		for i in range(len(attitude_pilot.angular_velocity_PIDs)):
			print '%.3f\t%.3f\t%.3f\t%.3f' % (attitude_pilot.angular_velocity_PIDs[i].PTerm, attitude_pilot.angular_velocity_PIDs[i].ITerm, attitude_pilot.angular_velocity_PIDs[i].DTerm, attitude_pilot.angular_velocity_PIDs[i].output)
		print
		for elem in attitude_pilot.max_angular_velocities:
			print '%.3f\t' % elem,
		print attitude_pilot.gain_schedule_scale
