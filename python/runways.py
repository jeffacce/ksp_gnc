import math
from ksp_constants import KERBIN_RADIUS


class runway:
	def __init__(self, centerline_eq_coefs, thresholds):
		self.centerline_eq_coefs = centerline_eq_coefs
		self.thresholds = thresholds

	# coords: (latitude, longitude)
	def get_centerline_deviation(self, coords):
		centerline_angular_deviation = (
			(
				self.centerline_eq_coefs[0] * coords[0]
				+ self.centerline_eq_coefs[1] * coords[1]
				+ self.centerline_eq_coefs[2]
			)
			/
			(	
				self.centerline_eq_coefs[0]**2
				+ self.centerline_eq_coefs[1]^2
			) ** 0.5
		)
		centerline_linear_deviation = -2 * math.pi * KERBIN_RADIUS * centerline_angular_deviation / 360

ksc_runway = runway(
	[
		142.13236295,
		1,
		81.62849024,
	],
	[
		(-0.0502118560109606, -74.4899977802028),
	]
)
