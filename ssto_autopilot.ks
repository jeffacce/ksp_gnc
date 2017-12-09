set CONFIG:IPU to 500.

set ALTITUDE_START_PITCHDOWN to 12000.
set SWITCH_ENGINE_THRUST_THRESHOLD to 900.
set TARGET_APOAPSIS to 102000.
set KERBIN_ATMOSPHERE_EDGE_ALTITUDE to 70000.
set GEAR_UP_POSITIVE_RATE_THRESHOLD to 5.

set TAKEOFF_ROLL_PITCH to 1.1.
    set D_PITCH_0_1 to 2.
set LOWER_ATMOSPHERE_ASCENT_PITCH to 15.
    set D_PITCH_1_2 to -0.5.
set PITCHDOWN_PITCH to 12.

set CENTERLINE_EQ_A to 142.13236295.    // latitude coefficient for the linear equation
set CENTERLINE_EQ_B to 1.               // longitude coefficient
set CENTERLINE_EQ_C to 81.62849024.     // constant term
set RUNWAY_EAST_THRESHOLD_LAT to -0.0502118560109606.
set RUNWAY_EAST_THRESHOLD_LNG to -74.4899977802028.
set CENTERLINE_PID_OUTPUT_LIMIT to 2.

set TIMESTEP to 0.1.

clearscreen.

// takeoff roll
set state to 0.
// state:
// 0 - takeoff roll
// 1 - lower atmosphere ascent
// 2 - pitchdown
// 3 - closed cycle power
// 4 - coast to edge of atmosphere
// 5 - circularization


print "Takeoff roll.".
set centerline_pid to pidloop(1, 0.2, 3, -CENTERLINE_PID_OUTPUT_LIMIT, CENTERLINE_PID_OUTPUT_LIMIT).
set centerline_pid:setpoint to 0.
lock current_heading_target to centerline_pid:output + 90.

set current_pitch_target to TAKEOFF_ROLL_PITCH.
lock centerline_angular_deviation to (CENTERLINE_EQ_A * ship:geoposition:lat + CENTERLINE_EQ_B * ship:geoposition:lng + CENTERLINE_EQ_C) / sqrt(CENTERLINE_EQ_A^2 + CENTERLINE_EQ_B^2).
lock centerline_linear_deviation to -2 * constant:pi * KERBIN:RADIUS * centerline_angular_deviation / 360.
lock steering to heading(current_heading_target, current_pitch_target).
lock throttle to 1.
stage.

when state = 0 then {
    clearscreen.
    print "Centerline deviation: " + centerline_linear_deviation.
    print "Target heading: " + current_heading_target.
    centerline_pid:update(time:seconds, centerline_linear_deviation).
    preserve.
}

wait until ship:geoposition:lat < RUNWAY_EAST_THRESHOLD_LAT and ship:geoposition:lng > RUNWAY_EAST_THRESHOLD_LNG.
set state to 1.
lock current_heading_target to 90.

if state = 1 {
    when ship:verticalspeed > GEAR_UP_POSITIVE_RATE_THRESHOLD then {
        print "Positive rate - gear up.".
        set gear to false.
    }
}

until current_pitch_target + D_PITCH_0_1 * TIMESTEP > LOWER_ATMOSPHERE_ASCENT_PITCH {
    clearscreen.
    set current_pitch_target to current_pitch_target + D_PITCH_0_1 * TIMESTEP.
    print current_pitch_target.
    wait TIMESTEP.
}
set current_pitch_target to LOWER_ATMOSPHERE_ASCENT_PITCH.
clearscreen.
print "Lower atmosphere ascent.".

wait until ship:altitude > ALTITUDE_START_PITCHDOWN.
print "Pitchdown.".
set state to 2.
until current_pitch_target + D_PITCH_0_1 * TIMESTEP < PITCHDOWN_PITCH {
    print current_pitch_target.
    set current_pitch_target to current_pitch_target + D_PITCH_1_2 * TIMESTEP.
    wait TIMESTEP.
}
set current_pitch_target to PITCHDOWN_PITCH.


wait until ship:maxthrust < SWITCH_ENGINE_THRUST_THRESHOLD.
print "Closed cycle power.".
set state to 3.
set ag10 to true.       // switch to closed cycle power

wait until ship:orbit:apoapsis > TARGET_APOAPSIS.
print "Coasting.".
set state to 4.
lock throttle to 0.
lock steering to ship:velocity:surface.

wait until ship:altitude > KERBIN_ATMOSPHERE_EDGE_ALTITUDE.
print "Edge of atmosphere.".
set state to 5.

// Configuration Coefficients

SET thrust_gain TO 10.
SET max_attitude_error to 5.
SET goal to 0.01 .

// Start continuous recalculation of controller variables.
// These will be reevalued when needed, and once they are
// connected to STEERING and THROTTLE, that will happen
// every physics tick.

LOCK circular_speed TO SQRT(BODY:MU/(BODY:RADIUS+ALTITUDE)).
LOCK horizontal_velocity TO VXCL(UP:VECTOR,VELOCITY:ORBIT).
LOCK circular_velocity TO horizontal_velocity:NORMALIZED*circular_speed.
LOCK remaining_burn TO circular_velocity-VELOCITY:ORBIT.
LOCK remaining_delta_v to remaining_burn:MAG.
LOCK max_accel TO MAXTHRUST/MASS.
LOCK attitude_error TO VANG(FACING:VECTOR,remaining_burn).
LOCK attitude_fade TO MAX(0,max_attitude_error-attitude_error)/max_attitude_error.

// Start steering in the needed direction, and
// set up thrust command. Initially our attitude is
// likely to be very wrong, so expect to observe
// the throttle being cut to zero until we are
// pointed in the right direction.

LOCK STEERING TO LOOKDIRUP(remaining_burn,facing:topvector).

// wait until near apoapsis.
lock max_acc to ship:maxthrust/ship:mass.
set burn_duration to remaining_delta_v/max_acc.
print "Ballpark burn estimate: " + round(burn_duration) + "s.".
WAIT UNTIL eta:apoapsis <= burn_duration / 2.
set state to 5.
print "Circularization.".

LOCK THROTTLE TO attitude_fade*thrust_gain*remaining_delta_v/max_accel.

// Hang out until we are done.
// NOTE: for some applications, this script could return
// to a calling script here. It would then be up to the
// calling script to keep hands off STEERING and THROTTLE
// until the job is done (or needs to be stopped), but
// would allow calling script to do other things.

WAIT UNTIL remaining_delta_v <= goal .

// Cut the throttle and stop steering.
// Calling script (if any) will reassert control.
// If called from keyboard, we leave the pilot
// throttle input at zero.

SET SHIP:CONTROL:PILOTMAINTHROTTLE TO 0.
UNLOCK THROTTLE.
UNLOCK STEERING.