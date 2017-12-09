import numpy as np
from utilities import angle_between

class telemetry_stream:
	def __init__(self, vessel, conn):
		self.conn = conn
		self.vessel = vessel
		self.reference_frame = vessel.orbit.body.reference_frame
		self.surf_vessel_reference_frame = conn.space_center.ReferenceFrame.create_hybrid(
											    position=vessel.orbit.body.reference_frame,
											    rotation=vessel.surface_reference_frame,
										   )
		self.vessel_met = conn.add_stream(getattr, vessel, 'met')
		self._vessel_vs = conn.add_stream(getattr, vessel.flight(), 'vertical_speed')
		self._vessel_mean_altitude = conn.add_stream(getattr, vessel.flight(), 'mean_altitude')
		self._vessel_surface_altitude = conn.add_stream(getattr, vessel.flight(), 'surface_altitude')
		self._vessel_surface_v = conn.add_stream(getattr, vessel.flight(self.surf_vessel_reference_frame), 'velocity')
		self._vessel_speed = conn.add_stream(getattr, vessel.flight(self.reference_frame), 'speed')
		self._vessel_aoa = conn.add_stream(getattr, vessel.flight(), 'angle_of_attack')
		self._vessel_thrust = conn.add_stream(getattr, vessel, 'thrust')
		self._vessel_max_thrust = conn.add_stream(getattr, vessel, 'max_thrust')
		self._vessel_available_torque = conn.add_stream(getattr, vessel, 'available_torque')
		self._vessel_moment_of_inertia = conn.add_stream(getattr, vessel, 'moment_of_inertia')
		self.vessel_attitude = [
			self.conn.add_stream(getattr, vessel.flight(), 'pitch'),
			self.conn.add_stream(getattr, vessel.flight(), 'roll'),
			self.conn.add_stream(getattr, vessel.flight(), 'sideslip_angle'),
		]
		self._vessel_Q = conn.add_stream(getattr, vessel.flight(), 'dynamic_pressure')
		self._last_met = None
		self._cached_vessel_vs = None
		self._cached_vessel_mean_altitude = None
		self._cached_vessel_surface_altitude = None
		self._cached_vessel_surface_v = None
		self._cached_vessel_speed = None
		self._cached_vessel_aoa = None
		self._cached_vessel_thrust = None
		self._cached_vessel_max_thrust = None
		self._cached_vessel_available_torque = None
		self._cached_vessel_moment_of_inertia = None
		self._cached_vessel_attitude = None
		self._cached_vessel_Q = None
		self._cached_vessel_surface_v_heading = None

	def time_elapsed(self):
		this_met = self.vessel_met()
		result = self._last_met == this_met
		self._last_met = this_met
		return result

	def update_all(self):
		self._cached_vessel_vs = self._vessel_vs()
		self._cached_vessel_mean_altitude = self._vessel_mean_altitude()
		self._cached_vessel_surface_altitude = self._vessel_surface_altitude()
		self._cached_vessel_surface_v = self._vessel_surface_v()
		self._cached_vessel_speed = self._vessel_speed()
		self._cached_vessel_aoa = self._vessel_aoa()
		self._cached_vessel_thrust = self._vessel_thrust()
		self._cached_vessel_max_thrust = self._vessel_max_thrust()
		self._cached_vessel_available_torque = self._vessel_available_torque()
		self._cached_vessel_moment_of_inertia = self._vessel_moment_of_inertia()
		self._cached_vessel_Q = self._vessel_Q()
		self._cached_vessel_surface_v_heading = self.vessel_surface_v_heading()

	def vessel_vs(self):
		if self._cached_vessel_vs is None or self.time_elapsed():
			self._cached_vessel_vs = self._vessel_vs()
		return self._cached_vessel_vs 
	def vessel_mean_altitude(self):
		if self._cached_vessel_mean_altitude is None or self.time_elapsed():
			self._cached_vessel_mean_altitude = self._vessel_mean_altitude()
		return self._cached_vessel_mean_altitude 
	def vessel_surface_altitude(self):
		if self._cached_vessel_surface_altitude is None or self.time_elapsed():
			self._cached_vessel_surface_altitude = self._vessel_surface_altitude()
		return self._cached_vessel_surface_altitude
	def vessel_surface_v(self):
		if self._cached_vessel_surface_v is None or self.time_elapsed():
			self._cached_vessel_surface_v = self._vessel_surface_v()
		return self._cached_vessel_surface_v 
	def vessel_speed(self):
		if self._cached_vessel_speed is None or self.time_elapsed():
			self._cached_vessel_speed = self._vessel_speed()
		return self._cached_vessel_speed 
	def vessel_aoa(self):
		if self._cached_vessel_aoa is None or self.time_elapsed():
			self._cached_vessel_aoa = self._vessel_aoa()
		return self._cached_vessel_aoa 
	def vessel_thrust(self):
		if self._cached_vessel_thrust is None or self.time_elapsed():
			self._cached_vessel_thrust = self._vessel_thrust()
		return self._cached_vessel_thrust 
	def vessel_max_thrust(self):
		if self._cached_vessel_max_thrust is None or self.time_elapsed():
			self._cached_vessel_max_thrust = self._vessel_max_thrust()
		return self._cached_vessel_max_thrust 
	def vessel_available_torque(self):
		if self._cached_vessel_available_torque is None or self.time_elapsed():
			self._cached_vessel_available_torque = self._vessel_available_torque()
		return self._cached_vessel_available_torque 
	def vessel_moment_of_inertia(self):
		if self._cached_vessel_moment_of_inertia is None or self.time_elapsed():
			self._cached_vessel_moment_of_inertia = self._vessel_moment_of_inertia()
		return self._cached_vessel_moment_of_inertia
	def vessel_Q(self):
		if self._cached_vessel_Q is None or self.time_elapsed():
			self._cached_vessel_Q = self._vessel_Q()
		return self._cached_vessel_Q
	def vessel_surface_v_heading(self):
		if self._cached_vessel_surface_v_heading is None or self.time_elapsed():
			# vessel surface reference frame: x-up, y-north, z-east.
			# ditching x because heading does not care about vertical speed.
			surface_velocity = np.array(self.vessel_surface_v())[1:]
			flat_north = np.array((1, 0))
			self._cached_vessel_surface_v_heading = np.rad2deg(angle_between(flat_north, surface_velocity))
		return self._cached_vessel_surface_v_heading
