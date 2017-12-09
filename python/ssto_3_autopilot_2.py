import krpc
import time
import os

from autopilot import autopilot
from runways import ksc_runway

GEAR_UP_POSITIVE_RATE_THRESHOLD = 5
TAKEOFF_ROLL_NEUTRAL_PITCH = 1.2
V_ROTATE = 150.0

ROTATE_RATE = 1.5		# wheel rotate rate in degrees per second
LOWER_ASCENT_PITCH = 30
PITCHDOWN_RATE = -2	# lower atmosphere ascent to final ascent pitch down rate
UPPER_ASCENT_PITCH = 15

ALTITUDE_START_PITCHDOWN = 8000
SWITCH_ENGINE_THRUST_THRESHOLD = 900
KERBIN_ATMOSPHERE_EDGE_ALTITUDE = 70000
APOAPSIS_FINE_TUNE_THROTTLE_GAIN = 0.5


TARGET_APOAPSIS = 100000


# phases:
# 'takeoff' - takeoff roll
# 'lower_ascent' - lower atmosphere ascent
# 'final_ascent' - final ascent
# 'coast' - coast to edge of atmosphere
# 'fine_tune_ap' - fine-tune apoapsis
# 'circularization' - circularization

# states:
# gear up/down depending on situations
#		positive rate - gear up
# closed cycle power when available thrust drops below threshold

# vessel.met (mission elapsed time) is equivalent to time:seconds


# new phases:
# takeoff, heading alignment, lower ascent, upper ascent, coast, fine tune apoapsis, circularization

#################
conn = krpc.connect(name='Sub-orbital flight')
vessel = conn.space_center.active_vessel


####### TAKEOFF ROLL #######
phase = 'takeoff'
ap = autopilot(vessel, conn)
ap.set_target(pitch=TAKEOFF_ROLL_NEUTRAL_PITCH, roll=0, throttle=1, sideslip=0)
vessel.control.activate_next_stage()
while phase == 'takeoff':
	os.system('clear')
	print 'Takeoff. V: %.1f m/s. V_rotate: %.1f m/s' % (ap.telemetry.vessel_speed(), V_ROTATE)
	ap.update()
	ap.steer()
	if ap.telemetry.vessel_speed() > V_ROTATE:
		phase = 'rotate'
		t_start = ap.telemetry.vessel_met()
		JATO_staged = False

while phase == 'rotate':
	os.system('clear')
	t_now = ap.telemetry.vessel_met()
	pitch_target = TAKEOFF_ROLL_NEUTRAL_PITCH + (t_now - t_start) * ROTATE_RATE
	print 'Rotate: %.1f degrees' % pitch_target
	ap.set_target(pitch=pitch_target)
	ap.update()
	ap.steer()
	if not JATO_staged and ap.telemetry.vessel_surface_altitude() > 200:
		vessel.control.activate_next_stage()
		JATO_staged = True
	if ap.telemetry.vessel_vs() > GEAR_UP_POSITIVE_RATE_THRESHOLD:
		vessel.control.gear()
	if pitch_target >= LOWER_ASCENT_PITCH:
		phase = 'lower_ascent'
		ap.set_target(pitch=LOWER_ASCENT_PITCH)
		ap.set_target(heading=90)

while phase == 'lower_ascent':
	os.system('clear')
	ap.update()
	ap.steer()
	if ap.telemetry.vessel_mean_altitude() > ALTITUDE_START_PITCHDOWN:
		phase = 'pitchdown'
		t_start = ap.telemetry.vessel_met()

while phase == 'pitchdown':
	os.system('clear')
	t_now = ap.telemetry.vessel_met()
	pitch_target = LOWER_ASCENT_PITCH + (t_now - t_start) * PITCHDOWN_RATE
	print 'Pitchdown: %.1f degrees' % pitch_target
	ap.set_target(pitch=pitch_target)
	ap.update()
	ap.steer()
	if pitch_target <= UPPER_ASCENT_PITCH:
		phase = 'upper_ascent'
		ap.set_target(pitch=UPPER_ASCENT_PITCH)

while phase == 'upper_ascent':
	os.system('clear')
	print 'Upper ascent.'
	ap.update()
	ap.steer()
	if vessel.orbit.apoapsis > TARGET_APOAPSIS:
		phase = 'coast'

