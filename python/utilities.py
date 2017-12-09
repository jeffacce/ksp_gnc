import krpc
import numpy as np

# TODO: rewrite vessel flight telemetry as a cache object of streams. Call rpc to check if physics tick happened; if not, use cached results

def get_angular_velocity(vessel, space_center):
	body_ref_frame = vessel.orbit.body.non_rotating_reference_frame
	angvel = vessel.angular_velocity(body_ref_frame)
	angvel = space_center.transform_direction(angvel, body_ref_frame, vessel.reference_frame)
	return angvel

# https://stackoverflow.com/questions/21483999/using-atan2-to-find-angle-between-two-vectors
def angle_between(v1, v2):
    angle = np.arctan2(v2[1], v2[0]) - np.arctan2(v1[1], v1[0])
    if (angle < 0):
    	angle += 2 * np.pi;
    return angle
