import krpc
import time
import os


from runways import ksc_runway


ENGINE_SPOOL_UP_THRUST_THRESHOLD = 950000
GEAR_UP_POSITIVE_RATE_THRESHOLD = 5
TAKEOFF_ROLL_NEUTRAL_PITCH = 1.2
D_PITCH_0_1 = 2		# takeoff roll to lower atmosphere pitch up rate
LOWER_ATMOSPHERE_ASCENT_PITCH = 35
D_PITCH_1_2 = -2	# lower atmosphere ascent to final ascent pitch down rate
FINAL_ASCENT_PITCH = 12

ALTITUDE_START_PITCHDOWN = 8000
SWITCH_ENGINE_THRUST_THRESHOLD = 900
KERBIN_ATMOSPHERE_EDGE_ALTITUDE = 70000
APOAPSIS_FINE_TUNE_THROTTLE_GAIN = 0.5


TARGET_APOAPSIS = 100000


# phases:
# 'spool' - engine spool-up
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


#################
conn = krpc.connect(name='Sub-orbital flight')
vessel = conn.space_center.active_vessel

####### ENGINE SPOOL-UP #######
phase = 'spool'
vessel.control.sas = False
vessel.control.activate_next_stage()
vessel.control.throttle = 1
while phase == 'spool':
	thrust_level_pctg = vessel.thrust / ENGINE_SPOOL_UP_THRUST_THRESHOLD * 100
	os.system('clear')
	print "Engine spool-up. Thrust: %.2f%%" % thrust_level_pctg
	if thrust_level_pctg > 100:
		phase = 'takeoff'


####### TAKEOFF ROLL #######
os.system('clear')
print 'Takeoff.'
# !!!! 90.55 needs to be moved into ksc_runway
vessel.auto_pilot.target_pitch_and_heading(TAKEOFF_ROLL_NEUTRAL_PITCH, 90.55)
vessel.auto_pilot.engage()
vessel.control.activate_next_stage()


## TODO: automatically steer down centerline


